"""Optional cooperative primary graph search; never dispatch Motion commands."""

from collections import OrderedDict
from copy import copy
import time

import mujoco
import numpy as np


def astar_settings(planning):
    settings = planning.get('astar', {})
    if not isinstance(settings, dict):
        raise ValueError('planning.astar must be a mapping')
    mode = settings.get('mode', 'blocking')
    if mode not in ('blocking', 'incremental'):
        raise ValueError('planning.astar.mode must be blocking or incremental')
    result = {'mode': mode}
    for name, default, minimum, maximum in (
        ('slice_budget_ms', 10., .1, 100.),
        ('slice_period_ms', 10., 1., 100.),
        ('max_operations_per_slice', 2048, 1, 10000),
    ):
        value = settings.get(name, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not np.isfinite(value) or not minimum <= value <= maximum):
            raise ValueError(f'Invalid planning.astar.{name}')
        if name == 'max_operations_per_slice' and int(value) != value:
            raise ValueError('max_operations_per_slice must be an integer')
        result[name] = int(value) if name == 'max_operations_per_slice' else float(value)
    return result


class GraphSearchWorkspace:
    """Reuse one private collision context while sharing the immutable roadmap.

    Live callbacks may change geometry while the generator is suspended. A
    native model copy and separate data prevent that from changing its scene.
    Neither a new process nor another graph is built for each search.
    """

    def __init__(self, planner):
        self.live_planner = planner
        self.planner = copy(planner)
        self.planner.goal_selector = planner.goal_selector.fork()
        checker = copy(planner.collision_filter)
        checker.model = copy(checker.model)
        checker.data = mujoco.MjData(checker.model)
        checker._human_report_cache = OrderedDict()
        self.planner.collision_filter = checker
        self.planner.planner = copy(planner.planner)
        self.planner.planner.checker = checker
        self.planner.planner.state_cache = {}
        self.planner.planner.edge_cache = {}

    def capture(self, live):
        self.planner.goal_selector = self.live_planner.goal_selector.fork()
        self.planner.goal_seed = (None if self.live_planner.goal_seed is None
                                  else self.live_planner.goal_seed.copy())
        self.planner.goal_empty_until = self.live_planner.goal_empty_until
        checker = self.planner.collision_filter
        for field in ('geom_size', 'geom_contype', 'geom_conaffinity'):
            getattr(checker.model, field)[:] = getattr(live.model, field)
        for field in ('mocap_pos', 'mocap_quat'):
            getattr(checker.data, field)[:] = getattr(live.data, field)
        for field in ('human_obstacles_enabled', 'human_status', 'human_capsules',
                      'human_received_at', 'human_expires_at', 'environment_revision',
                      'human_data_timeout', 'human_safety_margin', 'safety_margin',
                      'max_joint_step'):
            setattr(checker, field, getattr(live, field))
        checker._human_report_cache.clear()
        self.planner.planner.revision = None
        self.planner.current_path = None
        self.planner.path_revision = None
        self.planner.last_plan_failure = 'NONE'

    def same_scene(self, live):
        """Fresh timestamps alone do not change geometry or safety rules."""
        checker = self.planner.collision_filter
        return all(getattr(checker, field) == getattr(live, field) for field in (
            'human_obstacles_enabled', 'human_status', 'human_capsules',
            'human_safety_margin', 'human_data_timeout', 'safety_margin', 'max_joint_step',
        )) and all(np.array_equal(getattr(a, field), getattr(b, field)) for a, b, fields in (
            (checker.model, live.model, ('geom_size', 'geom_contype', 'geom_conaffinity')),
            (checker.data, live.data, ('mocap_pos', 'mocap_quat')),
        ) for field in fields)


class GraphSearch:
    def __init__(self, workspace, live_checker, start, target, generation,
                 goal_tolerance, target_clearance, settings, search_graph=True):
        workspace.capture(live_checker)
        self.workspace = workspace
        self.planner = workspace.planner
        self.start = np.asarray(start).copy()
        self.target = np.asarray(target).copy()
        self.generation = generation
        self.goal_tolerance = goal_tolerance
        self.target_clearance = target_clearance
        self.settings = settings
        self.search_graph = search_graph
        self.goal = None
        self.planned = False
        self.done = False
        self.error = None
        self.slices = 0
        self.last_slice_ms = 0.
        self.max_slice_ms = 0.
        self.work_ms = 0.
        self.operations_total = 0
        self.last_slice_operations = 0
        self.last_yield_reason = 'NOT_STARTED'
        self.time_budget_yields = 0
        self.operation_limit_yields = 0
        self.began = time.monotonic()
        self.steps = self._steps()

    def _steps(self):
        tried = []
        for goal in self.planner.goal_candidates_steps(
                self.target, self.goal_tolerance, self.target_clearance):
            yield
            if goal is None or any(np.array_equal(goal, previous) for previous in tried):
                continue
            self.goal = goal.copy()
            if not self.search_graph:
                self.planner.last_plan_failure = 'NONE'
                return
            tried.append(goal.copy())
            self.planned = yield from self.planner.plan_steps(self.start, goal)
            if self.planned or self.planner.last_plan_failure != 'NO_GRAPH_PATH':
                return
            if len(tried) >= self.planner.max_goal_candidates:
                return
        if self.goal is None:
            self.planner.last_plan_failure = (
                'GOAL_UNREACHABLE' if self.planner.human_data_ready()
                else 'HUMAN_VISION_UNKNOWN')
        elif not self.planner.human_data_ready():
            self.planner.last_plan_failure = 'HUMAN_VISION_UNKNOWN'

    def advance(self):
        if self.done:
            return
        began = time.perf_counter()
        deadline = began + self.settings['slice_budget_ms'] / 1000.
        self.slices += 1
        self.last_slice_operations = 0
        try:
            for _ in range(self.settings['max_operations_per_slice']):
                next(self.steps)
                self.last_slice_operations += 1
                if time.perf_counter() >= deadline:
                    self.last_yield_reason = 'TIME_BUDGET'
                    self.time_budget_yields += 1
                    break
            else:
                self.last_yield_reason = 'OPERATION_LIMIT'
                self.operation_limit_yields += 1
        except StopIteration:
            self.done = True
            self.last_yield_reason = 'COMPLETE'
        except Exception as error:
            self.done = True
            self.error = error
            self.last_yield_reason = 'ERROR'
            self.steps.close()
        finally:
            self.operations_total += self.last_slice_operations
            self.last_slice_ms = (time.perf_counter() - began) * 1000.
            self.max_slice_ms = max(self.max_slice_ms, self.last_slice_ms)
            self.work_ms += self.last_slice_ms

    def close(self):
        self.steps.close()

    def retry_reason(self, live_checker):
        """Explain invalid failed-search evidence without turning it into RRT admission."""
        if not self.planner.human_data_ready():
            if self.planner.collision_filter.human_status == 'OBSERVED':
                return 'SNAPSHOT_EXPIRED'
            return 'SNAPSHOT_DATA_UNAVAILABLE'
        if self.planner.last_plan_failure == 'HUMAN_VISION_UNKNOWN':
            return 'SNAPSHOT_DATA_UNAVAILABLE'
        if not self.workspace.same_scene(live_checker):
            return 'SCENE_CHANGED'
        return None

    def statistics(self, reason=None):
        return dict(slices=self.slices, last_slice_ms=round(self.last_slice_ms, 3),
                    task='GRAPH_SEARCH' if self.search_graph else 'GOAL_SELECTION',
                    max_slice_ms=round(self.max_slice_ms, 3), work_ms=round(self.work_ms, 3),
                    elapsed_ms=round((time.monotonic() - self.began) * 1000., 3),
                    operations_total=self.operations_total,
                    last_slice_operations=self.last_slice_operations,
                    last_yield_reason=self.last_yield_reason,
                    time_budget_yields=self.time_budget_yields,
                    operation_limit_yields=self.operation_limit_yields,
                    search_failure=self.planner.last_plan_failure if self.done else None,
                    reason=reason or (self.planner.last_plan_failure if self.done else 'PENDING'))
