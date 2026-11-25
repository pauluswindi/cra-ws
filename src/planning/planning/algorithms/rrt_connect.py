"""Bounded geometric RRT-Connect for shadow and simulation fallback workers.

Both trees use the existing checker, finite five-joint bounds and linear joint
segments. This does not predict obstacle motion or authorize margin recovery.
Do not run concurrently with other users of the same mutable collision checker.
"""

import time

import numpy as np


class _Tree:
    def __init__(self, root, capacity, from_start):
        self.points = np.empty((capacity, 5), dtype=float)
        self.parents = np.full(capacity, -1, dtype=int)
        self.points[0] = root
        self.size = 1
        self.from_start = from_start

    def root_path(self, index):
        indices = []
        while index >= 0:
            indices.append(index)
            index = int(self.parents[index])
        return self.points[indices[::-1]].copy()


class RRTConnectPlanner:
    """Search one snapshot, returning an N-by-5 path or None with a reason.

    Step size is Euclidean joint distance in radians, not human clearance.
    The time bound is cooperative: an individual checker call cannot be
    interrupted, but a result arriving after the deadline is never accepted.
    """

    def __init__(self, collision_filter, step_size=0.15, time_budget=0.5,
                 max_iterations=1000, max_nodes=3000, clock=None, trace=None):
        for name, value in [('step_size', step_size), ('time_budget', time_budget)]:
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0.:
                raise ValueError(f'{name} must be positive and finite')
        for name, value, minimum in [('max_iterations', max_iterations, 1),
                                     ('max_nodes', max_nodes, 2)]:
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f'{name} must be an integer >= {minimum}')
        self.checker = collision_filter
        self.lower = np.array(collision_filter.lower, dtype=float, copy=True)
        self.upper = np.array(collision_filter.upper, dtype=float, copy=True)
        if (self.lower.shape != (5,) or self.upper.shape != (5,)
                or not np.isfinite(self.lower).all() or not np.isfinite(self.upper).all()
                or np.any(self.lower >= self.upper)):
            raise ValueError('RRT-Connect requires finite ordered five-joint bounds')
        self.step_size = float(step_size)
        self.time_budget = float(time_budget)
        self.max_iterations = max_iterations
        self.max_nodes = max_nodes
        self.clock = time.perf_counter if clock is None else clock
        self.last_plan_failure = 'NOT_RUN'
        self.last_error = None
        self.last_statistics = {}
        self.trace = trace

    def _event(self, event, **fields):
        if self.trace is not None:
            self.trace.emit(event, iteration=self.last_statistics['iterations'], **fields)

    def _stop(self, reason):
        self.last_plan_failure = reason
        return False

    def _alive(self):
        if self.last_plan_failure != 'SEARCHING':
            return False
        if self.checker.environment_revision != self.revision:
            return self._stop('ENVIRONMENT_CHANGED')
        if self.checker.human_obstacles_enabled and not self.checker.human_data_ready():
            return self._stop('HUMAN_VISION_UNKNOWN')
        if self.clock() >= self.deadline:
            return self._stop('TIME_BUDGET_EXHAUSTED')
        return True

    def _query(self, kind, *arguments):
        if not self._alive():
            return False
        self.last_statistics[kind + '_checks'] += 1
        try:
            method = getattr(self.checker, 'is_' + kind + '_valid')
            valid = bool(method(*arguments))
        except Exception as error:
            self.last_error = f'{type(error).__name__}: {error}'
            return self._stop('COLLISION_CHECK_ERROR')
        alive = self._alive()
        if self.trace is not None and not (alive and valid):
            failure = getattr(self.checker, 'last_' + kind + '_validation_failure', None) or {}
            self.query_reason = (
                self.last_plan_failure if not alive else failure.get('reason')
                or (failure.get('state') or {}).get('reason')
                or ('STATE_INVALID' if kind == 'state' else 'EDGE_INVALID'))
        return alive and valid

    def _extend(self, tree, target):
        if not self._alive():
            return 'TRAPPED', None
        delta = tree.points[:tree.size] - target
        nearest = int(np.argmin(np.einsum('ij,ij->i', delta, delta)))
        name = 'start' if tree.from_start else 'goal'
        if self.trace is not None:
            self._event('NEAREST_SELECTED', tree=name, q=tree.points[nearest].tolist(),
                        node=nearest)
        difference = target - tree.points[nearest]
        distance = float(np.linalg.norm(difference))
        if distance <= 1e-12:
            return 'REACHED', nearest
        if self.last_statistics['nodes'] >= self.max_nodes:
            self._stop('NODE_LIMIT')
            return 'TRAPPED', None
        reached = distance <= self.step_size
        candidate = (target.copy() if reached else
                     tree.points[nearest] + difference * (self.step_size / distance))
        if not self._query('state', candidate):
            self._rejection(name, candidate, tree.points[nearest], 'state')
            return 'TRAPPED', None
        # Goal-tree edges must be certified in execution order toward its root.
        edge = ((tree.points[nearest], candidate) if tree.from_start
                else (candidate, tree.points[nearest]))
        if not self._query('edge', *edge):
            self._rejection(name, candidate, tree.points[nearest], 'edge')
            return 'TRAPPED', None
        index = tree.size
        tree.points[index] = candidate
        tree.parents[index] = nearest
        tree.size += 1
        self.last_statistics['nodes'] += 1
        if self.trace is not None:
            self.trace.node(name, candidate, nearest, self.last_statistics['iterations'])
        return ('REACHED' if reached else 'ADVANCED'), index

    def _rejection(self, tree, candidate, parent, check):
        if self.trace is not None:
            reason = (self.query_reason if self.last_plan_failure == 'SEARCHING'
                      else self.last_plan_failure)
            self.trace.reject(tree, candidate, parent, reason, check,
                              self.last_statistics['iterations'])

    def _connect(self, tree, target):
        name = 'start' if tree.from_start else 'goal'
        if self.trace is not None:
            self._event('CONNECT_STARTED', tree=name, q=target.tolist())
        status, index = self._extend(tree, target)
        while status == 'ADVANCED':
            if self.trace is not None:
                self._event('CONNECT_ADVANCED', tree=name, q=tree.points[index].tolist())
            status, index = self._extend(tree, target)
        if status == 'REACHED' and self.trace is not None:
            self._event('CONNECT_REACHED', tree=name, q=tree.points[index].tolist())
        return status, index

    def _finish(self, path):
        # Certify the assembled start-to-goal path too, including reversed
        # goal-tree branches. Never return a partial tree on a failed check.
        for point in path:
            if not self._query('state', point):
                if self._alive():
                    self._stop('PATH_INVALID')
                if self.trace is not None:
                    self._event('PATH_REJECTED', reason=self.query_reason
                                if self.last_plan_failure == 'PATH_INVALID'
                                else self.last_plan_failure, check='state')
                return None
        for a, b in zip(path[:-1], path[1:]):
            if not self._query('edge', a, b):
                if self._alive():
                    self._stop('PATH_INVALID')
                if self.trace is not None:
                    self._event('PATH_REJECTED', reason=self.query_reason
                                if self.last_plan_failure == 'PATH_INVALID'
                                else self.last_plan_failure, check='edge')
                return None
        if not self._alive():
            return None
        self.last_plan_failure = 'NONE'
        self.last_statistics['waypoints'] = len(path)
        self.last_statistics['path_length_rad'] = float(
            np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        if self.trace is not None:
            self.trace.path = path.tolist()
        return path.copy()

    def plan(self, start, goal, seed=0):
        """Build two temporary trees. A fixed seed reproduces the sampling.

        Failure due to a search limit is not a proof that no geometric path
        exists. Start/goal inside a human margin are rejected by normal rules.
        """
        began = self.clock()
        self.deadline = began + self.time_budget
        self.revision = self.checker.environment_revision
        self.last_plan_failure = 'SEARCHING'
        self.last_error = None
        self.last_statistics = {
            'iterations': 0, 'nodes': 0, 'state_checks': 0, 'edge_checks': 0,
            'waypoints': 0, 'path_length_rad': None,
        }
        if self.trace is not None:
            self.trace.reset()
        self._event('SEARCH_STARTED')
        try:
            try:
                start = np.array(start, dtype=float, copy=True)
                goal = np.array(goal, dtype=float, copy=True)
                if (start.shape != (5,) or goal.shape != (5,)
                        or not np.isfinite(start).all() or not np.isfinite(goal).all()
                        or isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
                    raise ValueError('Invalid configuration or random seed')
                if any(np.any(q < self.lower) or np.any(q > self.upper) for q in (start, goal)):
                    raise ValueError('Configuration outside sampling bounds')
            except (TypeError, ValueError):
                self._stop('INVALID_INPUT')
                return None
            if not self._query('state', start):
                self._rejection('start', start, start, 'state')
                if self._alive():
                    self._stop('START_INVALID')
                return None
            if self.trace is not None:
                self.trace.node('start', start)
            if not self._query('state', goal):
                self._rejection('goal', goal, goal, 'state')
                if self._alive():
                    self._stop('GOAL_INVALID')
                return None
            if self.trace is not None:
                self.trace.node('goal', goal)
            if np.array_equal(start, goal):
                return self._finish(start[None, :])
            self.last_statistics['nodes'] = 2
            if self._query('edge', start, goal):
                self._event('DIRECT_CONNECTION')
                return self._finish(np.vstack((start, goal)))
            self._rejection('start', goal, start, 'edge')
            if not self._alive():
                return None
            first = _Tree(start, self.max_nodes, True)
            second = _Tree(goal, self.max_nodes, False)
            random = np.random.default_rng(seed)
            for iteration in range(self.max_iterations):
                if not self._alive():
                    return None
                self.last_statistics['iterations'] = iteration + 1
                sample = random.uniform(self.lower, self.upper)
                if self.trace is not None:
                    self._event('SAMPLE_GENERATED', tree='start' if first.from_start else 'goal',
                                q=sample.tolist())
                status, index = self._extend(first, sample)
                if status != 'TRAPPED':
                    status, other = self._connect(second, first.points[index])
                    if status == 'REACHED':
                        a, b = first.root_path(index), second.root_path(other)
                        forward, backward = (a, b) if first.from_start else (b, a)
                        return self._finish(np.vstack((forward, backward[-2::-1])))
                first, second = second, first
            if self._alive():
                self._stop('ITERATION_LIMIT')
            return None
        finally:
            if self.trace is not None:
                self.trace.finish(self.trace.path if self.last_plan_failure == 'NONE' else None,
                                  self.last_plan_failure)
            self.last_statistics.update(
                reason=self.last_plan_failure, environment_revision=self.revision,
                elapsed_s=float(self.clock() - began), seed=seed,
            )
