"""Offline clearance preference around RRT-Connect; no runtime dispatch.

A preferred search band is not a new mandatory human margin. The original
checker is never reconfigured. All selected paths need ordinary certification.
"""

from collections import OrderedDict
import copy
import time

import numpy as np

from planning.algorithms.rrt_connect import RRTConnectPlanner
from planning.algorithms.collision_filter import CollisionFilter


def _samples(start, goal, step):
    count = max(1, int(np.ceil(np.abs(goal - start).max() / step)))
    for index in range(count + 1):
        yield start + (goal - start) * (index / count)


class _PreferredChecker:
    """Search guidance only. Delegate all mandatory checks to the real checker."""

    def __init__(self, checker, clearance, step):
        self.base = checker
        self.clearance = clearance
        self.step = step
        self.revision = checker.environment_revision
        self.search_checker = None
        if isinstance(checker, CollisionFilter):
            # Reuse immutable model/geometry, copy all mutable MuJoCo data
            # (including current mocap obstacle poses), and isolate the cache.
            # One stricter query is cheaper than repeating hard and soft FK.
            self.search_checker = object.__new__(_ClearanceGuidedFilter)
            self.search_checker.__dict__.update(checker.__dict__)
            self.search_checker.data = copy.copy(checker.data)
            self.search_checker._human_report_cache = OrderedDict()
            self.search_checker.preferred_clearance = clearance
            self.search_checker.max_joint_step = min(checker.max_joint_step, step)
            self.search_checker.human_report_cache_hits = 0
            self.search_checker.human_report_cache_misses = 0

    def __getattr__(self, name):
        return getattr(self.base, name)

    def human_data_ready(self):
        return (self.base.environment_revision == self.revision
                and self.base.human_data_ready()
                and (self.search_checker is None or self.search_checker.human_data_ready()))

    def is_state_valid(self, point):
        if self.search_checker is not None:
            return self.search_checker.is_state_valid(point)
        if not self.base.is_state_valid(point):
            return False
        report = self.base.human_collision_report(point)
        return (report['status'] == 'FAR' and report['clearance'] is not None
                and report['clearance'] >= self.clearance)

    def is_edge_valid(self, start, goal):
        if self.search_checker is not None:
            return self.search_checker.is_edge_valid(start, goal)
        revision = self.base.environment_revision
        if not self.base.is_edge_valid(start, goal):
            return False
        for point in _samples(start, goal, self.step):
            if not self.base.human_data_ready() or self.base.environment_revision != revision:
                return False
            # The full ordinary edge check above already covered mandatory
            # geometry. Extra samples measure preference only; candidate scoring
            # later recertifies hard geometry at this finer spacing too.
            report = self.base.human_collision_report(point)
            if (report['status'] != 'FAR' or report['clearance'] is None
                    or report['clearance'] < self.clearance):
                return False
        return self.base.human_data_ready() and self.base.environment_revision == revision


class _ClearanceGuidedFilter(CollisionFilter):
    """Always refine boxes, keeping the mandatory threshold unchanged.

    Fast boolean queries may return a sphere lower bound once it exceeds the
    margin. Preference scores instead require one consistent, refined metric.
    Both representations remain conservative; final hard certification is
    performed independently by the original checker.
    """

    def _human_collision_after_forward(self, include_pairs=False):
        report = super()._human_collision_after_forward(include_pairs=True)
        if (report['status'] == 'FAR' and report['clearance'] is not None
                and report['clearance'] < self.preferred_clearance):
            return dict(report, status='NEAR')
        return report


class ClearancePathPreference:
    """Improve one baseline path, preferring clearance within a length cap.

    The cooperative budget includes baseline scoring, one preferred RRT attempt,
    candidate scoring and final certification; baseline generation is external.
    This is bounded selection, not an optimal RRT* or a dynamic-human planner.
    """

    def __init__(self, checker, preferred_clearance=.07, max_length_ratio=1.25,
                 time_budget=2., search_budget=.8, sample_step=.01, clock=None):
        for name, value in [('preferred_clearance', preferred_clearance),
                            ('max_length_ratio', max_length_ratio),
                            ('time_budget', time_budget), ('search_budget', search_budget),
                            ('sample_step', sample_step)]:
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0.:
                raise ValueError(f'{name} must be positive and finite')
        if preferred_clearance < checker.human_safety_margin or max_length_ratio < 1.:
            raise ValueError('Preference cannot reduce the human margin or length ratio below 1')
        self.checker = checker
        self.preferred_clearance = float(preferred_clearance)
        self.max_length_ratio = float(max_length_ratio)
        self.time_budget = float(time_budget)
        self.search_budget = float(search_budget)
        self.sample_step = min(float(sample_step), float(checker.max_joint_step))
        self.clock = time.perf_counter if clock is None else clock
        self.last_plan_failure = 'NOT_RUN'
        self.last_error = None
        self.last_statistics = {}

    def _alive(self):
        if self.last_plan_failure != 'EVALUATING':
            return False
        if self.checker.environment_revision != self.revision:
            self.last_plan_failure = 'ENVIRONMENT_CHANGED'
        elif not self.checker.human_obstacles_enabled or not self.checker.human_data_ready():
            self.last_plan_failure = 'HUMAN_VISION_UNKNOWN'
        elif self.clock() >= self.deadline:
            self.last_plan_failure = 'TIME_BUDGET_EXHAUSTED'
        return self.last_plan_failure == 'EVALUATING'

    def _score(self, path):
        clearance = float('inf')
        start_clearance = None
        goal_clearance = None
        # The original checker validates every edge. Extra report samples use
        # the same or finer spacing to measure the path bottleneck, not its mean.
        for edge_index, (start, goal) in enumerate(zip(path[:-1], path[1:])):
            if not self._alive():
                return None
            valid = self.checker.is_edge_valid(start, goal)
            if not self._alive() or not valid:
                return None
            for sample_index, point in enumerate(_samples(start, goal, self.sample_step)):
                if not self._alive():
                    return None
                valid = self.checker.is_state_valid(point)
                report = (self.checker._human_collision_after_forward(include_pairs=True)
                          if isinstance(self.checker, CollisionFilter)
                          else self.checker.human_collision_report(point))
                self.last_statistics['clearance_samples'] += 1
                if not self._alive():
                    return None
                if (not valid or report['status'] != 'FAR' or report['clearance'] is None
                        or not np.isfinite(report['clearance'])):
                    return None
                clearance = min(clearance, float(report['clearance']))
                if edge_index == 0 and sample_index == 0:
                    start_clearance = float(report['clearance'])
                goal_clearance = float(report['clearance'])
        return {'min_clearance_m': clearance,
                'start_clearance_m': start_clearance, 'goal_clearance_m': goal_clearance,
                'length_rad': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())}

    def improve(self, baseline, seed=0):
        began = self.clock()
        self.deadline = began + self.time_budget
        self.revision = self.checker.environment_revision
        self.last_plan_failure = 'EVALUATING'
        self.last_error = None
        self.last_statistics = {'selection': None, 'candidate_reason': 'NOT_TRIED',
                                'clearance_samples': 0}
        try:
            try:
                path = np.array(baseline, dtype=float, copy=True)
                if (path.ndim != 2 or path.shape[1] != 5 or len(path) < 2
                        or not np.isfinite(path).all()
                        or isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
                    raise ValueError('Invalid baseline or seed')
                if not np.any(np.diff(path, axis=0)):
                    raise ValueError('Stationary input is not a movement path')
            except (TypeError, ValueError):
                self.last_plan_failure = 'INVALID_INPUT'
                return None
            baseline_score = self._score(path)
            if baseline_score is None:
                if self._alive():
                    self.last_plan_failure = 'BASELINE_INVALID'
                return None
            self.last_statistics['baseline_score'] = baseline_score
            selected, selection = path, 'BASELINE_RETAINED'
            # Fixed endpoints upper-bound any path's bottleneck clearance.
            # A tiny numerical allowance avoids rejecting an endpoint itself.
            preferred_clearance = max(self.checker.human_safety_margin, min(
                self.preferred_clearance, baseline_score['start_clearance_m'] - 1e-6,
                baseline_score['goal_clearance_m'] - 1e-6))
            self.last_statistics['effective_preference_m'] = preferred_clearance
            if baseline_score['min_clearance_m'] < preferred_clearance:
                # Leave half the remaining time for scoring and recertification.
                allowance = min(self.search_budget, (self.deadline - self.clock()) * .5)
                preferred = _PreferredChecker(
                    self.checker, preferred_clearance, self.sample_step)
                search = RRTConnectPlanner(preferred, time_budget=allowance, clock=self.clock)
                candidate = search.plan(path[0], path[-1], seed=seed)
                self.last_statistics['candidate_reason'] = search.last_plan_failure
                self.last_statistics['candidate_search'] = search.last_statistics.copy()
                if not self._alive():
                    return None
                if search.last_plan_failure == 'COLLISION_CHECK_ERROR':
                    self.last_plan_failure = search.last_plan_failure
                    self.last_error = search.last_error
                    return None
                if candidate is not None and (not np.array_equal(candidate[0], path[0])
                                              or not np.array_equal(candidate[-1], path[-1])):
                    candidate = None
                    self.last_statistics['candidate_reason'] = 'ENDPOINT_MISMATCH'
                if candidate is not None:
                    candidate_score = self._score(candidate)
                    if candidate_score is None:
                        if not self._alive():
                            return None
                        self.last_statistics['candidate_reason'] = 'CANDIDATE_INVALID'
                    else:
                        self.last_statistics['candidate_score'] = candidate_score
                        length_cap = self.max_length_ratio * baseline_score['length_rad'] + 1e-12
                        within_length = candidate_score['length_rad'] <= length_cap
                        improves = (candidate_score['min_clearance_m'] >
                                    baseline_score['min_clearance_m'] + 1e-9)
                        if within_length and improves:
                            selected, selection = candidate, 'CLEARER_PATH_SELECTED'
                        elif not within_length:
                            self.last_statistics['candidate_reason'] = 'LENGTH_CAP_EXCEEDED'
            else:
                self.last_statistics['candidate_reason'] = 'PREFERENCE_ALREADY_MET'
            final_score = self._score(selected)
            if final_score is None:
                if self._alive():
                    self.last_plan_failure = 'FINAL_PATH_INVALID'
                return None
            if not self._alive():
                return None
            self.last_statistics.update(selection=selection, selected_score=final_score)
            self.last_plan_failure = 'NONE'
            return selected.copy()
        except Exception as error:
            self.last_plan_failure = 'COLLISION_CHECK_ERROR'
            self.last_error = f'{type(error).__name__}: {error}'
            return None
        finally:
            self.last_statistics.update(reason=self.last_plan_failure,
                                        environment_revision=self.revision,
                                        elapsed_s=float(self.clock() - began))
