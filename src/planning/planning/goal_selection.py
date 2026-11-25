"""Bounded goal recovery in CRA base_link; never commands the robot."""

from copy import deepcopy
import time

import mujoco
import numpy as np

from simulation.frame_transform import FrameTransform


def goal_settings(config):
    section = config.get('planning', {}).get('goal_selection', {})
    if not isinstance(section, dict):
        raise ValueError('planning.goal_selection must be a mapping')
    ik = section.get('ik_fallback', {})
    if not isinstance(ik, dict):
        raise ValueError('goal_selection.ik_fallback must be a mapping')
    settings = {'retain_valid_goal': section.get('retain_valid_goal', False),
                'ik_enabled': ik.get('enabled', False)}
    for name, value in settings.items():
        if not isinstance(value, bool):
            raise ValueError(f'goal_selection.{name} must be boolean')
    for name, value, low, high, integer in (
        ('clearance_candidates', section.get('clearance_candidates', 1), 1, 16, True),
        ('preferred_clearance', section.get('preferred_clearance', .07), 0., 1., False),
        ('max_seeds', ik.get('max_seeds', 6), 1, 32, True),
        ('max_iterations', ik.get('max_iterations', 80), 1, 500, True),
        ('time_budget_ms', ik.get('time_budget_ms', 100.), .1, 500., False),
        ('retry_interval', ik.get('retry_interval', 1.), .1, 60., False),
        ('position_tolerance', ik.get('position_tolerance', .001), .00001, .01, False),
    ):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not np.isfinite(value) or not low <= value <= high
                or (integer and int(value) != value)):
            raise ValueError(f'Invalid goal_selection.{name}')
        settings[name] = int(value) if integer else float(value)
    inverse = config.get('kinematics', {}).get('inverse', {})
    for name, default in (('damping', .01), ('step_size', .5), ('max_joint_step', .1)):
        value = inverse.get(name, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not np.isfinite(value) or value <= 0.):
            raise ValueError(f'Invalid kinematics.inverse.{name}')
        settings[name] = float(value)
    # This new recovery remains a simulation experiment, like RRT fallback.
    settings['ik_enabled'] &= config.get('robot', {}).get('mode') == 'simulation'
    return settings


def position_and_jacobian(checker, body):
    """Translate world FK/Jacobian to the same CRA base frame as joint_eof."""
    conversion = (FrameTransform.ROBOT_FROM_MUJOCO_BASE
                  @ checker.data.xmat[checker.base_id].reshape(3, 3).T)
    point = conversion @ (checker.data.xpos[body] - checker.data.xpos[checker.base_id])
    jacobian = np.zeros((3, checker.model.nv))
    mujoco.mj_jacBody(checker.model, checker.data, jacobian, None, body)
    dofs = checker.model.jnt_dofadr[checker.joint_ids]
    return point, conversion @ jacobian[:, dofs]


def position(checker, body):
    conversion = (FrameTransform.ROBOT_FROM_MUJOCO_BASE
                  @ checker.data.xmat[checker.base_id].reshape(3, 3).T)
    return conversion @ (checker.data.xpos[body] - checker.data.xpos[checker.base_id])


def solve_position_steps(checker, body, point, seed, settings, alive):
    """Damped least-squares position IK with limits and a cooperative checkpoint.

    Intermediate mathematical iterates are not motion or collision certificates.
    The caller must validate the endpoint and the complete subsequent path.
    """
    q = np.clip(np.asarray(seed, dtype=float), checker.lower, checker.upper)
    previous = checker.data.qpos[checker.qpos_addresses].copy()
    try:
        for _ in range(settings['max_iterations']):
            yield
            if not alive():
                return None
            if not checker._set_joint_configuration(q):
                return None
            current, jacobian = position_and_jacobian(checker, body)
            error = point - current
            if np.linalg.norm(error) <= settings['position_tolerance']:
                return q.copy()
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + settings['damping'] ** 2 * np.eye(3), error)
            norm = np.linalg.norm(delta)
            if not np.isfinite(norm):
                return None
            if norm > settings['max_joint_step']:
                delta *= settings['max_joint_step'] / norm
            q = np.clip(q + settings['step_size'] * delta, checker.lower, checker.upper)
        return None
    finally:
        checker._set_joint_configuration(previous)


class GoalSelector:
    def __init__(self, config):
        self.settings = goal_settings(config)
        self.body_name = config['robot']['frames']['end_effector']
        self.key = None
        self.last_goal = None
        self.last_source = None
        self.cached_ik = []
        self.ik_revision = None
        self.ik_start = None
        self.next_ik_time = float('-inf')
        self.delayed_retry = False
        self.seed_offset = 0
        self.report = None

    def fork(self):
        return deepcopy(self)

    def adopt(self, other):
        self.__dict__.update(deepcopy(other.__dict__))

    def remember(self, goal):
        self.last_goal = np.asarray(goal).copy()
        is_ik = any(np.allclose(goal, q, atol=1e-6, rtol=0.) for q in self.cached_ik)
        self.last_source = 'IK' if is_ik else 'GRAPH'

    def retry_due(self, now):
        return self.delayed_retry and now >= self.next_ik_time

    def _context(self, target, tolerance, clearance):
        key = tuple(target) + (float(tolerance), float(clearance))
        if self.key != key:
            self.key = key
            self.last_goal = self.last_source = None
            self.cached_ik = []
            self.ik_revision = self.ik_start = None
            self.next_ik_time = float('-inf')
            self.delayed_retry = False
            self.seed_offset = 0

    def _valid(self, checker, q, source):
        self.report['checked'] += 1
        if checker.is_state_valid(q):
            self.report['valid_' + source] += 1
            return True
        failure = checker.last_state_validation_failure or {}
        reason = failure.get('reason', 'UNKNOWN_REJECTION')
        rejected = self.report['rejected']
        rejected[reason] = rejected.get(reason, 0) + 1
        if len(self.report['examples']) < 3:
            human = failure.get('human', {})
            self.report['examples'].append(dict(
                source=source, reason=reason, robot_geom=human.get('robot_geom'),
                clearance_m=human.get('clearance'), contact=failure.get('contact')))
        return False

    @staticmethod
    def _ik_ready(planner):
        checker = planner.collision_filter
        return (not planner.goal_require_human
                or (checker.human_obstacles_enabled and checker.human_data_ready())
                or (not checker.human_capsules and planner.goal_empty_until is not None
                    and time.monotonic() < planner.goal_empty_until))

    def candidates_steps(self, planner, target, tolerance, clearance):
        target = np.asarray(target, dtype=float)
        if (target.shape != (3,) or not np.isfinite(target).all()
                or not np.isfinite(tolerance) or not np.isfinite(clearance)
                or not 0. <= clearance < tolerance):
            raise ValueError('Invalid Cartesian goal window')
        self._context(target, tolerance, clearance)
        checker = planner.collision_filter
        revision = checker.environment_revision
        self.delayed_retry = False
        self.report = dict(reason='SEARCHING', graph_outcome=None, source=None,
                           window_candidates=0, checked=0, valid_graph=0, valid_ik=0,
                           valid_retained=0, rejected={}, examples=[], ik_attempts=0,
                           ik_cache_hits=0, ik_elapsed_ms=0.)

        def alive():
            if not planner.human_data_ready():
                self.report['reason'] = 'DATA_NOT_READY'
                return False
            if checker.environment_revision != revision:
                self.report['reason'] = 'ENVIRONMENT_CHANGED'
                return False
            return True

        if not alive():
            return
        body = checker.model.body(self.body_name).id
        distances = np.linalg.norm(planner.joint_eof - target, axis=1)
        ids = np.flatnonzero((distances >= clearance) & (distances <= tolerance))
        ids = ids[np.argsort(distances[ids], kind='stable')]
        self.report['window_candidates'] = len(ids)
        retained = None
        if self.settings['retain_valid_goal'] and self.last_goal is not None:
            yield
            if not alive():
                return
            if self._valid(checker, self.last_goal, 'retained'):
                distance = np.linalg.norm(position(checker, body) - target)
                if clearance <= distance <= tolerance:
                    retained = self.last_goal.copy()
                    self.report.update(reason='RETAINED_GOAL', source=self.last_source)
                    yield retained
        pool = []
        consumed = 0
        for consumed, index in enumerate(ids, 1):
            yield
            if not alive():
                return
            q = planner.joint_q[index]
            if retained is not None and np.array_equal(q, retained):
                continue
            if self._valid(checker, q, 'graph'):
                gap = getattr(checker, 'last_state_validation_clearance', None)
                score = self.settings['preferred_clearance'] if gap is None else min(
                    float(gap), self.settings['preferred_clearance'])
                pool.append((-score, float(distances[index]), int(index), q.copy()))
                if len(pool) >= self.settings['clearance_candidates']:
                    break
        if pool:
            self.report.update(reason='GRAPH_GOAL_SELECTED', graph_outcome='VALID', source='GRAPH')
            for _, _, _, q in sorted(pool):
                if not alive():
                    return
                yield q
            # Initial selection ranks a small pool, but existing alternative-goal
            # callers must still be able to continue through the remaining graph.
            for index in ids[consumed:]:
                yield
                if not alive():
                    return
                q = planner.joint_q[index]
                if retained is not None and np.array_equal(q, retained):
                    continue
                if self._valid(checker, q, 'graph'):
                    yield q.copy()
            return
        if retained is not None:
            return
        rejected = self.report['rejected']
        if not len(ids):
            graph_reason = 'NO_GRAPH_SAMPLES'
        elif set(rejected) == {'HUMAN_COLLISION'}:
            graph_reason = 'GRAPH_GOALS_BLOCKED_HUMAN'
        elif set(rejected).issubset({'MUJOCO_CONTACT', 'INVALID_JOINTS'}):
            graph_reason = 'GRAPH_GOALS_BLOCKED_STATIC_OR_LIMITS'
        else:
            graph_reason = 'GRAPH_GOALS_REJECTED_MIXED'
        self.report.update(reason=graph_reason, graph_outcome=graph_reason)
        if not self.settings['ik_enabled']:
            return
        if not self._ik_ready(planner):
            self.report['reason'] = 'IK_INPUT_NOT_READY'
            return
        for q in self.cached_ik:
            yield
            if not alive():
                return
            if self._valid(checker, q, 'ik') and clearance <= np.linalg.norm(
                    position(checker, body) - target) <= tolerance:
                self.report.update(reason='CACHED_IK_GOAL', source='IK')
                self.report['ik_cache_hits'] += 1
                yield q.copy()
                return
        seed = (planner.goal_seed.copy() if planner.goal_seed is not None
                else checker.data.qpos[checker.qpos_addresses].copy())
        moved = self.ik_start is None or np.max(np.abs(seed - self.ik_start)) > .01
        if self.ik_revision == revision and not moved:
            self.report['reason'] = 'IK_SCENE_ALREADY_TRIED'
            return
        if time.monotonic() < self.next_ik_time:
            self.report['reason'] = 'IK_RETRY_COOLDOWN'
            self.delayed_retry = True
            return
        self.ik_revision, self.ik_start = revision, seed.copy()
        self.next_ik_time = time.monotonic() + self.settings['retry_interval']
        # Bounded diverse initial guesses; no random sampling per camera frame.
        nearest = np.argsort(distances, kind='stable')[:64]
        seeds = [seed]
        offsets = np.array([[0., .6, -.6, .6, 0.], [0., -.6, .6, -.6, 0.],
                            [.7, .3, -.4, .2, .7], [-.7, -.3, .4, -.2, -.7],
                            [0., 0., 0., .8, -.8]])
        for attempt in range(self.settings['max_seeds'] - 1):
            index = max(nearest, key=lambda i: min(
                np.linalg.norm(planner.joint_q[i] - q) for q in seeds))
            q = planner.joint_q[index].copy()
            if min(np.linalg.norm(q - old) for old in seeds) < .2:
                q = np.clip(seed + offsets[attempt % len(offsets)], checker.lower, checker.upper)
            seeds.append(q)
        points = []
        if len(ids):
            points.append(planner.joint_eof[ids[0]].copy())
        checker._set_joint_configuration(seed)
        direction = position(checker, body) - target
        norm = np.linalg.norm(direction)
        if norm > 1e-9:
            points.append(target + direction / norm * ((clearance + tolerance) / 2.))
        points.extend(target + axis * ((clearance + tolerance) / 2.) for axis in np.eye(3))
        began = time.perf_counter()
        deadline = began + self.settings['time_budget_ms'] / 1000.

        def ik_alive():
            return alive() and self._ik_ready(planner) and time.perf_counter() < deadline

        found = None
        try:
            for attempt in range(self.settings['max_seeds']):
                if not ik_alive():
                    break
                self.report['ik_attempts'] += 1
                q = yield from solve_position_steps(
                    checker, body, points[(attempt // 2) % len(points)],
                    seeds[(attempt + self.seed_offset) % len(seeds)], self.settings, ik_alive)
                if not ik_alive():
                    break
                if q is not None:
                    yield
                    if not ik_alive():
                        break
                    if self._valid(checker, q, 'ik') and clearance <= np.linalg.norm(
                            position(checker, body) - target) <= tolerance:
                        if not ik_alive():
                            break
                        if not any(np.allclose(q, old, atol=1e-3, rtol=0.)
                                   for old in self.cached_ik):
                            self.cached_ik = (self.cached_ik + [q.copy()])[-4:]
                        self.report.update(reason='IK_GOAL_SELECTED', source='IK')
                        found = q.copy()
                        break
            if found is None and self.report['reason'] not in (
                    'DATA_NOT_READY', 'ENVIRONMENT_CHANGED'):
                self.report['reason'] = ('IK_BUDGET_EXHAUSTED' if time.perf_counter() >= deadline
                                         else 'IK_NO_VALID_GOAL')
        finally:
            self.seed_offset += self.report['ik_attempts']
            self.report['ik_elapsed_ms'] = round((time.perf_counter() - began) * 1000., 3)
        if found is not None:
            yield found
