"""Shared isolated RRT jobs and the observation-only coordinator.

The fallback coordinator also reuses these jobs; this module never publishes
Motion commands. Each job sees one copied scene with its original monotonic
expiry. New observations never extend it.
"""

from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import os
import time

import numpy as np

from planning.algorithms.clearance_preference import ClearancePathPreference
from planning.algorithms.collision_filter import CollisionFilter
from planning.algorithms.rrt_connect import RRTConnectPlanner
from planning.path_planner import PathPlanner
from planning.rrt_trace import SearchTrace
from planning.visualization.projection import project_configuration


_CHECKER = None


def scene_expiry(checker, scene):
    """Keep the original capture deadline, including a certified EMPTY scene."""
    if 'human_valid_until' in scene:
        return scene['human_valid_until']
    received = scene['human_received_at']
    return (float('-inf') if received is None else
            min(received + checker.human_data_timeout,
                scene['human_expires_at'] or float('inf')))


def scene_ready(checker, scene):
    if scene.get('human_data_kind') == 'EMPTY':
        return (not scene['human_obstacles_enabled'] and not scene['human_capsules']
                and time.monotonic() < scene_expiry(checker, scene))
    return checker.human_obstacles_enabled and checker.human_data_ready()


def capture_scene(checker):
    """Copy all mutable obstacle arrays and retain original freshness times."""
    return {
        'revision': checker.environment_revision,
        'mocap_pos': checker.data.mocap_pos.copy(),
        'mocap_quat': checker.data.mocap_quat.copy(),
        'geom_size': checker.model.geom_size.copy(),
        'geom_contype': checker.model.geom_contype.copy(),
        'geom_conaffinity': checker.model.geom_conaffinity.copy(),
        'human_obstacles_enabled': checker.human_obstacles_enabled,
        'human_status': checker.human_status,
        'human_capsules': checker.human_capsules,
        'human_received_at': checker.human_received_at,
        'human_expires_at': checker.human_expires_at,
    }


def _initialize(model_path, joint_names, checker_options):
    global _CHECKER
    _CHECKER = CollisionFilter(model_path, joint_names, **checker_options)


def _restore(checker, scene):
    for field in ('geom_size', 'geom_contype', 'geom_conaffinity'):
        getattr(checker.model, field)[:] = scene[field]
    for field in ('mocap_pos', 'mocap_quat'):
        getattr(checker.data, field)[:] = scene[field]
    for field in ('human_obstacles_enabled', 'human_status', 'human_capsules',
                  'human_received_at', 'human_expires_at'):
        setattr(checker, field, scene[field])
    checker.environment_revision = scene['revision']
    checker._human_report_cache.clear()


def run_job(job, checker=None):
    """Run only in the worker (optional checker permits deterministic tests)."""
    checker = _CHECKER if checker is None else checker
    began = time.perf_counter()
    trace = None
    result = {'kind': job['kind'], 'worker_pid': os.getpid(), 'reason': 'WORKER_ERROR',
              'path': None, 'scene_revision': job['scene']['revision']}
    try:
        _restore(checker, job['scene'])
        checker.max_joint_step = job['sample_step']
        if not scene_ready(checker, job['scene']):
            result['reason'] = 'HUMAN_VISION_UNKNOWN'
            return result
        remaining = job.get('request_deadline', float('inf')) - time.monotonic()
        if remaining <= 0.:
            result['reason'] = 'TIME_BUDGET_EXHAUSTED'
            return result
        if job['kind'] == 'search':
            trace = SearchTrace() if job.get('visualize', False) else None
            search = RRTConnectPlanner(
                checker, time_budget=min(job['search_budget'], remaining), trace=trace)
            baseline = search.plan(job['start'], job['goal'], seed=job['seed'])
            result.update(reason=search.last_plan_failure, search=search.last_statistics)
            if baseline is not None and not job.get('use_preference', True):
                result['path'] = baseline
            elif baseline is not None:
                preference = ClearancePathPreference(
                    checker, preferred_clearance=job['preferred_clearance'],
                    time_budget=job['selection_budget'],
                    search_budget=job['preference_search_budget'])
                selected = preference.improve(baseline, seed=job['seed'])
                result.update(reason=preference.last_plan_failure, path=selected,
                              preference=preference.last_statistics)
        elif job['kind'] == 'validate':
            # Reuse CRA's actual densification/waypoint limit and certification.
            # No graph load or node publishing is needed for these methods.
            planner = object.__new__(PathPlanner)
            planner.collision_filter = checker
            planner.max_segment_delta = job['max_segment_delta']
            path = np.array(job['path'], dtype=float, copy=True)
            path[0] = job['start']
            prepared = planner.prepare_execution_path(path)
            result.update(reason='NONE', points=len(prepared))
            if job.get('return_prepared', False):
                result['prepared_path'] = prepared
        else:
            raise ValueError('Unknown shadow job kind')
        if not scene_ready(checker, job['scene']):
            result.update(reason='HUMAN_VISION_UNKNOWN', path=None, prepared_path=None)
        elif (time.perf_counter() - began >= job['total_budget']
              or time.monotonic() >= job.get('request_deadline', float('inf'))):
            result.update(reason='TIME_BUDGET_EXHAUSTED', path=None, prepared_path=None)
    except Exception as error:
        result.update(reason='VALIDATION_FAILED' if job['kind'] == 'validate'
                      else 'WORKER_ERROR', error=f'{type(error).__name__}: {error}')
        result['failure'] = checker.last_path_validation_failure if checker else None
        if checker is not None and not scene_ready(checker, job['scene']):
            result['reason'] = 'HUMAN_VISION_UNKNOWN'
        elif time.perf_counter() - began >= job.get('total_budget', float('inf')):
            result['reason'] = 'TIME_BUDGET_EXHAUSTED'
    finally:
        result['elapsed_s'] = time.perf_counter() - began
        if trace is not None:
            # Projection happens after search/selection, in this worker's private data.
            # A visualization error must never change the search result.
            try:
                base, eof = job.get('projection_bodies', ('base_link', 'end_effector'))
                projected = trace.project(lambda q: project_configuration(
                    checker.model, checker.data, checker.qpos_addresses,
                    checker.model.body(base).id, checker.model.body(eof).id, q))
                result['visualization'] = dict(
                    projected, scene_revision=job['scene']['revision'],
                    expires_at=scene_expiry(checker, job['scene']),
                    human_data_kind=job['scene'].get('human_data_kind', 'OBSERVED'),
                    reason=search.last_plan_failure, projection_s=time.perf_counter() - began
                    - result['elapsed_s'])
            except Exception as error:
                result['visualization_error'] = f'{type(error).__name__}: {error}'
    return result


class ShadowRRTObserver:
    """One in-flight job, no backlog, no live checker mutation or command API.

    Human geometry may change during search: validate a candidate on a NEW
    snapshot in the same isolated process. A change during validation rejects
    its report as current; there is no repeated validation loop or auto-resume.
    """

    def __init__(self, model_path, joint_names, checker_options, *,
                 search_budget=.5, selection_budget=2., preference_search_budget=.8,
                 validation_budget=.5, preferred_clearance=.07, retry_interval=3.,
                 executor=None, projection_bodies=('base_link', 'end_effector')):
        settings = dict(search_budget=search_budget, selection_budget=selection_budget,
                        preference_search_budget=preference_search_budget,
                        validation_budget=validation_budget,
                        preferred_clearance=preferred_clearance, retry_interval=retry_interval)
        if any(isinstance(value, bool) or not np.isfinite(value) or value <= 0
               for value in settings.values()):
            raise ValueError('Shadow budgets, preference and retry interval must be positive')
        if preferred_clearance < checker_options['human_safety_margin']:
            raise ValueError('Shadow preference cannot reduce mandatory human margin')
        self.settings = settings
        self.sample_step = min(.01, checker_options['max_joint_step'])
        self.executor = executor if executor is not None else ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context('spawn'),
            initializer=_initialize,
            initargs=(str(model_path), list(joint_names), dict(checker_options)))
        self.future = None
        self.job = None
        self.attempt = None
        self.last_sequence = 0
        self.last_started = float('-inf')
        self.discard_reason = None
        self.disabled = False
        self.last_skip = None
        self.projection_bodies = tuple(projection_bodies)
        self.visualization_requested = False
        self.visualization_result = None

    @staticmethod
    def _context_reason(manager, attempt, start):
        if (manager.target_base is None
                or manager.target_generation != attempt['target_generation']):
            return 'TARGET_CHANGED'
        latest = manager.last_graph_failure
        if latest is None or latest['sequence'] != attempt['sequence']:
            return 'PLANNING_ATTEMPT_SUPERSEDED'
        if (manager.fault or manager.motion_active or manager.pending_action is not None
                or manager.motion_status not in ('IDLE', 'COMPLETED', 'STOPPED')
                or manager.safe_recovery_active or manager.safe_position_active
                or manager.human_margin_recovery_active):
            return 'MOTION_CONTEXT_CHANGED'
        if not manager.feedback_ready():
            return 'FEEDBACK_STALE'
        if not manager.obstacle_data_ready():
            return 'OBSTACLES_STALE'
        if np.max(np.abs(manager.current_q - start)) > .01:
            return 'START_MOVED'
        checker = manager.planner.collision_filter
        if not checker.human_obstacles_enabled or not checker.human_data_ready():
            return 'HUMAN_VISION_UNKNOWN'
        return None

    def _submit(self, job):
        self.job = job
        self.future = self.executor.submit(run_job, job)

    def tick(self, manager):
        """Poll in a short ROS callback. Returned dicts are diagnostics only."""
        if self.disabled:
            return None
        try:
            return self._tick(manager)
        except Exception as error:
            # A broken worker cannot change live Planning/Motion decisions.
            self.disabled = True
            return {'event': 'DISABLED', 'reason': 'WORKER_ERROR',
                    'error': f'{type(error).__name__}: {error}', 'observation_only': True}

    def _tick(self, manager):
        now = time.monotonic()
        if self.future is not None:
            reason = self._context_reason(manager, self.attempt, self.job['start'])
            if self.discard_reason is None:
                self.discard_reason = reason
            if not self.future.done():
                return None
            result = self.future.result()
            self.future = None
            visualization = result.pop('visualization', None)
            if visualization is not None:
                result['visualization_summary'] = dict(
                    projected_nodes=sum(len(n) for n in visualization['trees'].values()),
                    recorded_events=len(visualization['events']),
                    projection_s=visualization['projection_s'],
                    dropped_nodes=visualization['dropped_nodes'],
                    dropped_events=visualization['dropped_events'])
            if visualization is not None and self.discard_reason is None:
                self.visualization_result = dict(
                    visualization, target_generation=self.attempt['target_generation'])
            path = result.pop('path', None)
            event = dict(result, event='RESULT', attempt=self.attempt['sequence'],
                         observation_only=True, candidate_points=0 if path is None else len(path),
                         capture_to_result_s=now - self.job['captured_at'],
                         feedback_age_s=now - manager.feedback_time
                         if manager.feedback_time is not None else None)
            event['human_age_at_capture_s'] = (
                self.job['captured_at'] - self.job['scene']['human_received_at'])
            event['human_age_at_result_s'] = now - self.job['scene']['human_received_at']
            if self.discard_reason:
                event.update(event='DISCARDED', disposition=self.discard_reason)
            elif result['kind'] == 'search' and result['reason'] == 'NONE' and path is not None:
                if not (np.array_equal(path[0], self.job['start'])
                        and np.array_equal(path[-1], self.job['goal'])):
                    event.update(event='DISCARDED', disposition='ENDPOINT_MISMATCH')
                    return event
                # Do not reuse the old search scene or refresh its expiry.
                scene = capture_scene(manager.planner.collision_filter)
                self._submit(dict(kind='validate', scene=scene,
                                  captured_at=now, sample_step=self.sample_step,
                                  start=manager.current_q.copy(), path=path,
                                  max_segment_delta=manager.planner.max_segment_delta,
                                  total_budget=self.settings['validation_budget']))
                event['event'] = 'LATEST_VALIDATION_SUBMITTED'
            elif result['kind'] == 'validate':
                checker = manager.planner.collision_filter
                if result['reason'] != 'NONE':
                    event['disposition'] = 'LATEST_VALIDATION_FAILED'
                elif checker.environment_revision != self.job['scene']['revision']:
                    event['disposition'] = 'ENVIRONMENT_CHANGED_AFTER_VALIDATION'
                elif np.max(np.abs(manager.current_q - self.job['start'])) > 1e-9:
                    event['disposition'] = 'START_CHANGED_AFTER_VALIDATION'
                elif now >= min(self.job['scene']['human_received_at']
                                + checker.human_data_timeout,
                                self.job['scene']['human_expires_at'] or float('inf')):
                    event['disposition'] = 'VALIDATION_SNAPSHOT_EXPIRED'
                else:
                    event['disposition'] = 'VALID_FOR_CURRENT_SCENE_NO_DISPATCH'
            return event

        attempt = manager.last_graph_failure
        if (attempt is None or attempt['sequence'] <= self.last_sequence
                or now - self.last_started < self.settings['retry_interval']):
            return None
        reason = self._context_reason(manager, attempt, attempt['start'])
        if reason:
            skip = (attempt['sequence'], reason)
            if self.last_skip != skip:
                self.last_skip = skip
                return {'event': 'SKIPPED', 'attempt': attempt['sequence'],
                        'reason': reason, 'observation_only': True}
            return None
        self.attempt = dict(attempt)
        self.last_sequence = attempt['sequence']
        self.last_started = now
        self.discard_reason = None
        self._submit(dict(kind='search', scene=capture_scene(manager.planner.collision_filter),
                          captured_at=now, sample_step=self.sample_step,
                          start=manager.current_q.copy(), goal=attempt['goal'].copy(),
                          seed=attempt['sequence'],
                          visualize=self.visualization_requested,
                          projection_bodies=self.projection_bodies,
                          total_budget=self.settings['search_budget']
                          + self.settings['selection_budget'], **self.settings))
        return {'event': 'SEARCH_SUBMITTED', 'attempt': attempt['sequence'],
                'observation_only': True, 'scene_revision': self.job['scene']['revision']}

    def close(self):
        self.disabled = True
        if self.future is not None:
            self.future.cancel()
        self.executor.shutdown(wait=False, cancel_futures=True)
