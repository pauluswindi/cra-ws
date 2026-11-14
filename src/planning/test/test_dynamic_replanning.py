"""Exercise CRA replanning with the actual model, graph and collision checks.

Motion acknowledgements are supplied explicitly; these are not ROS transport
or C++ controller tests. Freeze monotonic time for a stable obstacle snapshot
during offline searches, and advance it explicitly for timeout checks.
"""

import contextlib
import io
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from planning.human_capsule_monitor import (
    HumanCapsule, HumanCapsuleMonitor, HumanSnapshot,
)
from planning.path_planner import PathPlanner
from planning.planning_manager import PlanningManager
from planning.algorithms.human_collision import nearest_human_collision, point_segment_distance
from simulation.frame_transform import FrameTransform


ROOT = Path(__file__).resolve().parents[3]
FAR = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
    HumanCapsule(0, 0, (10., 10., 10.), (10., 10., 11.), .1),
))
EMPTY = HumanSnapshot('EMPTY', 'NO_ACTIVE_CAPSULES', ())
UNKNOWN = HumanSnapshot('UNKNOWN', 'STALE_MESSAGE', ())
START = np.array([.023162707686424255, -1.3472504615783691,
                  1.1848934888839722, -.5477003455162048, .6865991950035095])
GOAL = np.array([1.699122667312622, -1.0410857200622559,
                 .6139736175537109, -1.2174338102340698, .8341435790061951])
DETOUR = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
    HumanCapsule(0, 0, (.2008049280692695, -.15740321298645774,
                       -.44067871295904015),
                 (.2008049280692695, -.15740321298645774,
                  -.36067871295904014), .03),
))


class DynamicReplanningTests(unittest.TestCase):
    """Keep real geometry checks while driving the Motion status protocol."""

    def setUp(self):
        self.clock = patch('time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.output = io.StringIO()
        self.redirect = contextlib.redirect_stdout(self.output)
        self.redirect.__enter__()
        self.addCleanup(self.redirect.__exit__, None, None, None)

    def make_manager(self, snapshot=FAR, joints=None, margin_recovery=False):
        planner = PathPlanner(
            ROOT / 'src/planning/data/workspace_graph.npz',
            ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml',
            ROOT / 'src/cra/config/robot.yaml',
        )
        checker = planner.collision_filter
        base = checker.data.xpos[checker.base_id].copy()
        rotation = checker.data.xmat[checker.base_id].reshape(3, 3).copy()
        target_world = checker.data.mocap_pos[
            checker.obstacle_geoms['target']['mocap_id']].copy()
        target = FrameTransform.mujoco_base_to_robot_base(
            rotation.T @ (target_world - base))
        manager = PlanningManager(
            planner, execute_path=True, human_avoidance_enabled=True,
            simulation_only_ignore_unknown=True,
            human_margin_recovery_enabled=margin_recovery)
        manager.update_joint_state(np.zeros(5) if joints is None else joints)
        manager.update_obstacles([])
        manager.update_target(target, target_world)
        manager.update_motion_status('IDLE')
        manager.update_human_snapshot(snapshot)
        return manager

    @staticmethod
    def block_goal(manager):
        target = manager.target_base
        manager.update_human_snapshot(HumanSnapshot(
            'OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(0, 0, tuple(target),
                             tuple(target + [0., 0., .2]), .1),)))

    @staticmethod
    def stopped(manager):
        manager.pending_action = 'STOP'
        manager.pending_since = time.monotonic()
        manager.stop_owned = True
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())

    def test_no_human_and_far_human_can_execute_and_complete(self):
        for snapshot in (EMPTY, FAR):
            with self.subTest(status=snapshot.status):
                manager = self.make_manager(snapshot)
                self.assertEqual(manager.step().status, 'PATH_READY')
                decision = manager.step()
                self.assertEqual(decision.action, 'EXECUTE')
                self.assertTrue(manager.planner.collision_filter.is_path_valid(decision.path))
                manager.update_motion_status('EXECUTING')
                self.assertEqual(manager.step().status, 'EXECUTING')
                manager.update_motion_status('COMPLETED')
                self.assertEqual(manager.step().status, 'COMPLETED')

    def test_goal_is_retried_after_human_moves_away(self):
        manager = self.make_manager()
        self.block_goal(manager)
        self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
        manager.update_human_snapshot(FAR)
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.assertEqual(manager.step().action, 'EXECUTE')

    def test_unchanged_blocked_goal_is_not_searched_again(self):
        manager = self.make_manager()
        self.block_goal(manager)
        with patch.object(manager.planner, 'select_goal',
                          wraps=manager.planner.select_goal) as select:
            self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
            for _ in range(3):
                self.assertEqual(manager.step().action, 'NONE')
            self.assertEqual(select.call_count, 1)

    def test_ignored_unknown_does_not_repeat_same_failed_search(self):
        manager = self.make_manager(EMPTY)
        with patch.object(manager.planner, 'plan', return_value=False) as plan, \
                patch.object(manager, '_prepare_safe_position', return_value=False) as stage:
            manager.planner.last_plan_failure = 'NO_GRAPH_PATH'
            self.assertEqual(manager.step().status, 'NO_PATH')
            attempts = plan.call_count
            self.assertEqual(attempts, manager.planner.max_goal_candidates)
            for _ in range(3):
                manager.update_human_snapshot(EMPTY)
                manager.step()
            self.assertEqual(plan.call_count, attempts)
            self.assertEqual(stage.call_count, 1)

    def test_goal_retries_once_when_unknown_policy_changes_environment(self):
        manager = self.make_manager()
        self.block_goal(manager)
        self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
        manager.update_human_snapshot(UNKNOWN)
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.assertFalse(manager.planner.collision_filter.human_obstacles_enabled)
        # Fresh observed obstruction must prevent execution of this preview.
        self.block_goal(manager)
        self.assertEqual(manager.step().status, 'HUMAN_PATH_BLOCKED')
        self.assertFalse(manager.path_sent)

    def test_stop_ack_requires_a_new_joint_measurement(self):
        manager = self.make_manager()
        self.assertEqual(manager.step().status, 'PATH_READY')
        manager.pending_action = 'STOP'
        manager.pending_since = time.monotonic()
        manager.stop_owned = True
        manager.update_motion_status('STOPPED')
        decision = manager.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(decision.status, 'WAITING_FOR_FEEDBACK')
        self.assertEqual(manager.human_report['reason'], 'WAITING_FOR_FEEDBACK')
        manager.update_joint_state(np.zeros(5))
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.assertEqual(manager.step().action, 'RELEASE')

    def test_invalid_prepared_trajectory_does_not_release_stop(self):
        manager = self.make_manager(EMPTY)
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.stopped(manager)
        with patch.object(manager.planner, 'prepare_execution_path',
                          side_effect=ValueError('execution path invalid')):
            decision = manager.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(decision.status, 'TRAJECTORY_ERROR')
        self.assertTrue(manager.stop_owned)
        self.assertEqual(manager.motion_status, 'STOPPED')
        self.assertIn('execution path invalid', manager.last_error)

    def test_release_ack_is_required_before_execution(self):
        manager = self.make_manager()
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.stopped(manager)
        self.assertEqual(manager.step().action, 'RELEASE')
        for _ in range(2):
            self.assertEqual(manager.step().action, 'NONE')
            self.assertEqual(manager.pending_action, 'RELEASE')
        manager.update_motion_status('IDLE')
        self.assertEqual(manager.step().action, 'EXECUTE')

    def test_new_obstruction_during_release_requests_stop(self):
        manager = self.make_manager()
        manager.step()
        self.stopped(manager)
        self.assertEqual(manager.step().action, 'RELEASE')
        self.block_goal(manager)
        self.assertEqual(manager.step().action, 'STOP')
        # A delayed IDLE acknowledgement must not clear the newer STOP.
        manager.update_motion_status('IDLE')
        self.assertEqual(manager.pending_action, 'STOP')
        self.assertEqual(manager.step().status, 'WAITING_FOR_STOP')

    def test_missing_release_ack_times_out_to_stop(self):
        manager = self.make_manager(EMPTY)
        manager.step()
        self.stopped(manager)
        self.assertEqual(manager.step().action, 'RELEASE')
        self.now.return_value = 101.9
        manager.update_joint_state(np.zeros(5))
        manager.update_obstacles([])
        self.now.return_value = 102.1
        decision = manager.step()
        self.assertEqual(decision.action, 'STOP')
        self.assertEqual(decision.status, 'MOTION_RESPONSE_TIMEOUT')

    def test_no_goal_remains_stopped_then_resumes_after_change(self):
        manager = self.make_manager()
        manager.step()
        manager.step()
        manager.update_motion_status('EXECUTING')
        self.block_goal(manager)
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(np.zeros(5))
        self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
        for _ in range(3):
            self.assertEqual(manager.step().action, 'NONE')
            self.assertTrue(manager.stop_owned)
        manager.update_human_snapshot(FAR)
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.assertEqual(manager.step().action, 'RELEASE')
        manager.update_motion_status('IDLE')
        self.assertEqual(manager.step().action, 'EXECUTE')

    def test_invalid_joint_input_is_not_reported_as_unknown_vision(self):
        manager = self.make_manager()
        manager.update_joint_state([0., 0., -.001, 0., 0.])
        decision = manager.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(decision.status, 'START_INVALID')

    def test_human_collision_keeps_invalid_start_stopped(self):
        manager = self.make_manager()
        checker = manager.planner.collision_filter
        checker._set_joint_configuration(manager.current_q)
        geom = checker.model.geom('link2_visual').id
        base = checker.data.xpos[checker.base_id]
        rotation = checker.data.xmat[checker.base_id].reshape(3, 3)
        center = FrameTransform.mujoco_base_to_robot_base(
            rotation.T @ (checker.data.geom_xpos[geom] - base))
        manager.update_human_snapshot(HumanSnapshot(
            'OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(0, 0, tuple(center), tuple(center + [0, 0, .1]), .05),)))
        with patch.object(manager.planner, 'plan', wraps=manager.planner.plan) as plan:
            for _ in range(3):
                decision = manager.step()
                self.assertEqual(decision.status, 'HUMAN_TOO_CLOSE')
                self.assertEqual(decision.action, 'NONE')
            self.assertEqual(plan.call_count, 0)
        report = manager.human_report
        self.assertEqual(report['status'], 'NEAR')
        self.assertEqual(report['joints'], manager.current_q.tolist())
        self.assertEqual(report['revision'], checker.environment_revision)
        for field in ('clearance', 'robot_geom', 'robot_radius', 'body_id',
                      'segment_id', 'human_radius', 'safety_margin'):
            self.assertIn(field, report)

    def test_planning_exception_is_available_for_diagnostics(self):
        manager = self.make_manager()
        with patch.object(manager.planner, 'select_goal',
                          side_effect=RuntimeError('test planner failure')):
            self.assertEqual(manager.step().status, 'PLANNING_ERROR')
        self.assertIn('RuntimeError', manager.last_error)
        self.assertIn('test planner failure', manager.last_error)

    def test_human_report_exception_stops_active_motion_with_diagnostics(self):
        manager = self.make_manager()
        manager.step()
        manager.step()
        manager.update_motion_status('EXECUTING')
        with patch.object(manager.planner.collision_filter, 'human_collision_report',
                          side_effect=RuntimeError('geometry query failed')):
            decision = manager.step()
        self.assertEqual(decision.action, 'STOP')
        self.assertIn('geometry query failed', manager.last_error)

    def test_feedback_expiring_during_validation_does_not_release_stop(self):
        manager = self.make_manager(EMPTY)
        manager.step()
        self.stopped(manager)
        prepare = manager.planner.prepare_execution_path

        def expire_feedback(path):
            result = prepare(path)
            self.now.return_value = 101.1
            return result

        with patch.object(manager.planner, 'prepare_execution_path',
                          side_effect=expire_feedback):
            decision = manager.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(decision.status, 'WAITING_FOR_FEEDBACK')
        self.assertTrue(manager.stop_owned)

    def test_identical_snapshots_preserve_revision_and_changed_geometry_does_not(self):
        manager = self.make_manager()
        checker = manager.planner.collision_filter
        revision = checker.environment_revision
        manager.update_human_snapshot(FAR)
        self.assertEqual(checker.environment_revision, revision)
        moved = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(0, 0, (10.01, 10., 10.), (10., 10., 11.), .1),))
        manager.update_human_snapshot(moved)
        self.assertGreater(checker.environment_revision, revision)

    def test_complete_frame_chain_agrees_with_graph_and_human_centers(self):
        manager = self.make_manager(joints=START)
        checker = manager.planner.collision_filter
        checker._set_joint_configuration(START)
        base = checker.data.xpos[checker.base_id].copy()
        rotation = checker.data.xmat[checker.base_id].reshape(3, 3).copy()
        conversion = FrameTransform.ROBOT_FROM_MUJOCO_BASE
        np.testing.assert_allclose(conversion.T @ conversion, np.eye(3))
        self.assertAlmostEqual(np.linalg.det(conversion), 1.)
        for index in range(0, len(manager.planner.joint_q), 100):
            checker._set_joint_configuration(manager.planner.joint_q[index])
            world = checker.data.xpos[checker.model.body('end_effector').id]
            expected = conversion @ rotation.T @ (world - base)
            np.testing.assert_allclose(expected, manager.planner.joint_eof[index], atol=1e-6)
        names = {checker.model.geom(i).name for i in checker.robot_geoms}
        self.assertEqual(names, {f'link{i}_visual' for i in range(1, 6)})
        checker._set_joint_configuration(START)
        geom = max(checker.robot_geoms, key=lambda i: checker.model.geom_rbound[i])
        center = conversion @ rotation.T @ (checker.data.geom_xpos[geom] - base)
        manager.update_human_snapshot(HumanSnapshot(
            'OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(0, 0, tuple(center), tuple(center + [0, 0, .1]), .05),)))
        report = checker.human_collision_report(START)
        self.assertEqual(report['status'], 'NEAR')
        self.assertAlmostEqual(report['center_distance'], 0., places=10)

    def test_human_monitor_preserves_status_and_timeout_reasons(self):
        monitor = HumanCapsuleMonitor()

        def point(value):
            return SimpleNamespace(x=value[0], y=value[1], z=value[2])
        message = SimpleNamespace(
            valid=True, header=SimpleNamespace(
                frame_id='base_link', stamp=SimpleNamespace(sec=10, nanosec=0)),
            capsules=[SimpleNamespace(body_id=0, segment_id=0, radius=.1,
                                      start=point((10, 10, 10)), end=point((10, 10, 11)))])
        self.assertTrue(monitor.update(message, 10_000_000_000))
        self.assertEqual(monitor.snapshot(10_000_000_000).status, 'OBSERVED')
        self.assertEqual(monitor.snapshot(10_600_000_000).reason, 'STALE_MESSAGE')
        self.now.return_value = 100.6
        self.assertEqual(monitor.snapshot(10_600_000_000).reason, 'MESSAGE_TIMEOUT')
        message.header.stamp.sec = 11
        self.assertTrue(monitor.update(message, 11_000_000_000))
        self.assertEqual(monitor.snapshot(11_000_000_000).status, 'OBSERVED')
        message.header.stamp.sec = 12
        message.capsules = []
        self.assertTrue(monitor.update(message, 12_000_000_000))
        self.assertEqual(monitor.snapshot(12_000_000_000).status, 'EMPTY')
        message.valid = False
        self.assertFalse(monitor.update(message, 12_000_000_000))
        self.assertEqual(monitor.snapshot(12_000_000_000).reason, 'VISION_INVALID')

    def test_expired_search_is_not_reported_as_graph_connectivity_failure(self):
        manager = self.make_manager()
        planner = manager.planner
        checker = planner.collision_filter
        goal = planner.select_goal(manager.target_base, .12, .04)

        def expire_during_search(start, goal):
            self.now.return_value = 100.6
            return None

        with patch.object(planner.planner, 'plan', side_effect=expire_during_search), \
                patch.object(planner, 'find_alternative_path') as fallback:
            self.assertEqual(manager.step().status, 'HUMAN_VISION_UNKNOWN')
            self.assertFalse(fallback.called)
        self.assertEqual(planner.last_plan_failure, 'HUMAN_VISION_UNKNOWN')
        self.assertIsNone(planner.planner.revision)
        revision = checker.environment_revision
        manager.update_joint_state(np.zeros(5))
        manager.update_obstacles([])
        manager.update_human_snapshot(FAR)
        self.assertEqual(checker.environment_revision, revision)
        self.assertIsNotNone(goal)
        self.assertEqual(manager.step().status, 'PATH_READY')

    def test_feedback_change_can_retry_invalid_start_without_geometry_change(self):
        manager = self.make_manager()
        manager.update_joint_state([0., 0., -.001, 0., 0.])
        self.assertEqual(manager.step().status, 'START_INVALID')
        manager.update_joint_state([0., -.02, .02, 0., 0.])
        self.assertEqual(manager.step().status, 'PATH_READY')

    def test_expired_goal_selection_is_not_reported_as_unreachable(self):
        manager = self.make_manager()

        def expire_goal(*args):
            self.now.return_value = 100.6
            return None

        with patch.object(manager.planner, 'select_goal', side_effect=expire_goal):
            self.assertEqual(manager.step().status, 'HUMAN_VISION_UNKNOWN')

    def test_expired_final_path_validation_cannot_return_ready(self):
        manager = self.make_manager()
        planner = manager.planner
        goal = planner.select_goal(manager.target_base, .12, .04)
        validate = planner.is_path_valid

        def expire_final(path):
            valid = validate(path)
            self.now.return_value = 100.6
            return valid

        with patch.object(planner, 'is_path_valid', side_effect=expire_final):
            self.assertFalse(planner.plan(np.zeros(5), goal))
        self.assertEqual(planner.last_plan_failure, 'HUMAN_VISION_UNKNOWN')
        self.assertIsNone(planner.current_path)

    def test_existing_astar_finds_detour_and_manager_resumes_after_stop(self):
        manager = self.make_manager(joints=START)
        planner = manager.planner
        checker = planner.collision_filter
        # Pin a known joint goal to isolate graph search and the Motion protocol.
        with patch.object(planner, 'select_goal', return_value=GOAL.copy()):
            self.assertEqual(manager.step().status, 'PATH_READY')
            original = manager.current_path.copy()
            self.assertEqual(manager.step().action, 'EXECUTE')
            manager.update_motion_status('EXECUTING')
            manager.update_human_snapshot(DETOUR)
            self.assertTrue(checker.is_state_valid(START))
            self.assertTrue(checker.is_state_valid(GOAL))
            self.assertFalse(checker.is_path_valid(original))
            self.assertEqual(manager.step().action, 'STOP')
            manager.update_motion_status('STOPPED')
            manager.update_joint_state(START)
            began = time.perf_counter()
            self.assertEqual(manager.step().status, 'PATH_READY')
            duration = time.perf_counter() - began
            alternate = manager.current_path.copy()
            old_length = np.linalg.norm(np.diff(original, axis=0), axis=1).sum()
            new_length = np.linalg.norm(np.diff(alternate, axis=0), axis=1).sum()
            self.assertGreater(new_length, old_length)
            self.assertTrue(checker.is_path_valid(alternate))
            clearances = []
            for a, b in zip(alternate[:-1], alternate[1:]):
                steps = max(1, int(np.ceil(np.max(np.abs(b - a)) / checker.max_joint_step)))
                for alpha in np.linspace(0., 1., steps + 1):
                    report = checker.human_collision_report(a + alpha * (b - a))
                    clearances.append(report['clearance'])
            minimum_clearance = min(clearances)
            self.assertGreater(minimum_clearance, checker.human_safety_margin)
            self.assertEqual(manager.step().action, 'RELEASE')
            manager.update_motion_status('IDLE')
            decision = manager.step()
            self.assertEqual(decision.action, 'EXECUTE')
            self.assertTrue(checker.is_path_valid(decision.path))
            manager.update_motion_status('EXECUTING')
            self.assertEqual(manager.step().status, 'EXECUTING')
        print(f'Offline detour: {old_length:.3f} -> {new_length:.3f} rad; '
              f'replanning={duration:.3f}s; min_clearance={minimum_clearance:.4f}m; '
              f'local={planner.planner.last_used_local_connections}',
              file=sys.stderr)

    def margin_manager(self, clearance=.075):
        joints = np.array([1.0831362616291125, -.7116884074291262,
                           1.2431678147673046, -.15903829199563738, .3179308568765186])
        manager = self.make_manager(joints=joints, margin_recovery=True)
        checker = manager.planner.collision_filter
        checker._set_joint_configuration(joints)
        base = checker.data.xpos[checker.base_id].copy()
        rotation = checker.data.xmat[checker.base_id].reshape(3, 3).copy()
        geom = checker.model.geom('link5_visual').id
        center = FrameTransform.mujoco_base_to_robot_base(
            rotation.T @ (checker.data.geom_xpos[geom] - base))
        # Place the capsule at the requested measured proxy clearance. Its old
        # sphere-based offset can now lie in proven empty space beside the mesh.
        lower, upper = 0., .5
        for _ in range(40):
            distance = (lower + upper) * .5
            point = center + [distance, 0, 0]
            snapshot = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(1, 6, tuple(point), tuple(point), .05),))
            manager.update_human_snapshot(snapshot)
            actual = checker.human_collision_report(joints)['clearance']
            if actual < clearance:
                lower = distance
            else:
                upper = distance
        return manager, snapshot

    def execute_margin_retreat(self):
        manager, snapshot = self.margin_manager()
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('STOPPED')
        self.assertEqual(manager.step().status, 'WAITING_FOR_FEEDBACK')
        manager.update_joint_state(manager.current_q.copy())
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')
        path = manager.current_path.copy()
        self.assertEqual(manager.step().action, 'RELEASE')
        manager.update_motion_status('IDLE')
        decision = manager.step()
        self.assertEqual(decision.action, 'EXECUTE')
        self.assertEqual(decision.status, 'HUMAN_RETREAT_SENT')
        manager.update_motion_status('EXECUTING')
        return manager, snapshot, path

    def test_margin_retreat_requires_all_pairs_to_keep_their_clearance(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter
        start = np.zeros(5)
        goal = np.array([.01, 0, 0, 0, 0])

        def states(joints):
            progress = joints[0] / .01
            return {'status': 'NEAR', 'clearance': .07 + .01 * progress,
                    'pairs': {(0, 0, 0): .07 + .01 * progress,
                              (1, 0, 0): .09 - .005 * progress}}

        # The closest pair improves, but another close pair worsens.
        with patch.object(checker, 'human_margin_state', side_effect=states):
            self.assertFalse(checker.is_human_margin_transition_valid(start, goal))

    def test_margin_retreat_never_enters_a_previously_clear_pair_buffer(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter

        def states(joints):
            progress = joints[0] / .01
            return {'status': 'NEAR', 'clearance': .07 + .01 * progress,
                    'pairs': {(0, 0, 0): .07 + .01 * progress,
                              (1, 0, 0): .101 - .005 * progress}}

        with patch.object(checker, 'human_margin_state', side_effect=states):
            self.assertFalse(checker.is_human_margin_transition_valid(
                np.zeros(5), np.array([.01, 0, 0, 0, 0])))

    def test_real_margin_path_has_monotonic_clearance_and_safe_endpoint(self):
        manager, _ = self.margin_manager()
        planner = manager.planner
        checker = planner.collision_filter
        start = manager.current_q.copy()
        path = planner.find_human_margin_retreat(start)
        self.assertIsNotNone(path)
        self.assertFalse(checker.is_state_valid(path[0]))
        self.assertFalse(checker.is_path_valid(path))
        self.assertTrue(checker.is_state_valid(path[-1]))
        self.assertTrue(checker.is_human_margin_transition_valid(path[0], path[-1]))
        self.assertFalse(checker.is_human_margin_transition_valid(
            start, start + [.01, 0, 0, 0, 0]))
        self.assertLessEqual(np.max(np.abs(path[-1] - path[0])), planner.max_joint_delta)
        previous = checker.human_margin_state(start)
        for alpha in np.linspace(0, 1, 101):
            state = checker.human_margin_state(path[0] + alpha * (path[-1] - path[0]))
            self.assertIsNotNone(state)
            for pair, value in state['pairs'].items():
                self.assertGreaterEqual(
                    value + 1e-9, min(previous['pairs'][pair], checker.human_safety_margin))
            previous = state
        self.assertGreater(previous['clearance'], checker.human_safety_margin)
        self.assertIsNone(planner.find_human_margin_retreat(start, max_steps=1))

    def test_margin_retreat_returns_to_normal_astar_and_original_target(self):
        manager, _, path = self.execute_margin_retreat()
        target = manager.target_base.copy()
        checker = manager.planner.collision_filter
        manager.update_joint_state(np.mean(path, axis=0))
        self.assertFalse(checker.is_state_valid(manager.current_q))
        self.assertEqual(manager.step().status, 'EXECUTING')
        manager.update_joint_state(path[-1])
        decision = manager.step()
        self.assertEqual((decision.action, decision.status), ('STOP', 'HUMAN_RETREAT_CLEAR'))
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(path[-1])
        with patch.object(manager.planner.planner, 'plan',
                          wraps=manager.planner.planner.plan) as astar:
            self.assertEqual(manager.step().status, 'PATH_READY')
            self.assertTrue(astar.called)
        np.testing.assert_array_equal(manager.target_base, target)
        self.assertFalse(manager.human_margin_recovery_active)
        self.assertEqual(manager.step().action, 'RELEASE')
        manager.update_motion_status('IDLE')
        decision = manager.step()
        self.assertEqual(decision.action, 'EXECUTE')
        self.assertTrue(checker.is_path_valid(decision.path))

    def test_unknown_during_margin_retreat_latches_stop_until_observed(self):
        manager, snapshot, _ = self.execute_margin_retreat()
        manager.update_human_snapshot(UNKNOWN)
        decision = manager.step()
        self.assertEqual((decision.action, decision.status),
                         ('STOP', 'HUMAN_RETREAT_DATA_UNKNOWN'))
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        with patch.object(manager.planner, 'plan', wraps=manager.planner.plan) as plan:
            for _ in range(3):
                self.assertEqual(manager.step().status, 'HUMAN_RETREAT_DATA_UNKNOWN')
            self.assertFalse(plan.called)
        self.assertTrue(manager.simulation_only_ignore_unknown)
        self.now.return_value = 100.6
        manager.update_human_snapshot(snapshot)
        manager.update_joint_state(manager.current_q.copy())
        manager.update_obstacles([])
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')

    def test_safe_new_human_revision_can_continue_a_margin_retreat(self):
        manager, snapshot, path = self.execute_margin_retreat()
        checker = manager.planner.collision_filter
        revision = checker.environment_revision
        capsule = snapshot.capsules[0]
        point = tuple(np.asarray(capsule.start) + [.002, 0, 0])
        manager.update_human_snapshot(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 6, point, point, capsule.radius),)))
        manager.update_joint_state(np.mean(path, axis=0))
        self.assertGreater(checker.environment_revision, revision)
        self.assertEqual(manager.step().status, 'EXECUTING')
        self.assertTrue(manager.human_margin_recovery_active)

    def test_overlap_during_margin_retreat_requests_stop(self):
        manager, snapshot, _ = self.execute_margin_retreat()
        capsule = snapshot.capsules[0]
        manager.update_human_snapshot(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 6, capsule.start, capsule.end, capsule.radius + .15),)))
        decision = manager.step()
        self.assertEqual((decision.action, decision.status), ('STOP', 'HUMAN_RETREAT_UNSAFE'))
        self.assertFalse(manager.human_margin_recovery_active)
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        manager.update_human_snapshot(UNKNOWN)
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_DATA_UNKNOWN')

    def test_measured_approach_during_margin_retreat_requests_stop(self):
        manager, _, _ = self.execute_margin_retreat()
        manager.update_joint_state(manager.current_q + [.01, 0, 0, 0, 0])
        decision = manager.step()
        self.assertEqual((decision.action, decision.status), ('STOP', 'HUMAN_RETREAT_UNSAFE'))
        self.assertIn('reduced human clearance', manager.last_error)

    def test_overlap_start_remains_stopped_without_a_retreat(self):
        manager, _ = self.margin_manager(clearance=-.01)
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        checker = manager.planner.collision_filter
        self.assertIsNone(checker.human_margin_state(manager.current_q))
        self.assertIsNone(manager.planner.find_human_margin_retreat(manager.current_q))
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
        self.assertTrue(manager.stop_owned)
        manager.update_human_snapshot(UNKNOWN)
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_DATA_UNKNOWN')

    def test_mujoco_contact_cannot_use_human_margin_retreat(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter
        checker._set_joint_configuration(manager.current_q)
        center = checker.data.geom_xpos[checker.model.geom('link5_visual').id].copy()
        checker.set_obstacle_pose('obstacle_1', center)
        self.assertNotEqual(checker.contact_report(manager.current_q)['status'], 'CLEAR')
        self.assertIsNone(manager.planner.find_human_margin_retreat(manager.current_q))

    def test_invalid_joint_and_unknown_data_cannot_use_margin_certificate(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter
        invalid = manager.current_q.copy()
        invalid[2] = -.001
        self.assertIsNone(checker.human_margin_state(invalid))
        self.assertFalse(checker.is_human_margin_transition_valid(invalid, manager.current_q))
        manager.update_human_snapshot(UNKNOWN)
        self.assertIsNone(checker.human_margin_state(manager.current_q))
        self.assertIsNone(manager.planner.find_human_margin_retreat(manager.current_q))

    def test_failed_margin_search_is_bounded_until_meaningful_change(self):
        manager, snapshot = self.margin_manager()
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        with patch.object(manager.planner, 'find_human_margin_retreat',
                          return_value=None) as search:
            for _ in range(3):
                self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
            self.assertEqual(search.call_count, 1)
            capsule = snapshot.capsules[0]
            shifted = tuple(np.asarray(capsule.start) + [.001, 0, 0])
            moved = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(1, 6, shifted, shifted, capsule.radius),))
            manager.update_human_snapshot(moved)
            self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
            self.assertEqual(search.call_count, 1)
            self.now.return_value = 100.6
            manager.update_human_snapshot(moved)
            manager.update_joint_state(manager.current_q.copy())
            manager.update_obstacles([])
            self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
            self.assertEqual(search.call_count, 2)

    def test_margin_recovery_does_not_override_an_external_stop(self):
        manager, _ = self.margin_manager()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        self.assertFalse(manager.stop_owned)
        self.assertEqual(manager.step().action, 'NONE')
        self.assertFalse(manager.human_margin_recovery_active)

    def test_new_unsafe_geometry_during_retreat_release_cannot_execute(self):
        manager, snapshot = self.margin_manager()
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        manager.step()
        self.assertEqual(manager.step().action, 'RELEASE')
        capsule = snapshot.capsules[0]
        manager.update_human_snapshot(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 6, capsule.start, capsule.end, capsule.radius + .15),)))
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('IDLE')
        self.assertEqual(manager.step().status, 'WAITING_FOR_STOP')

    def test_margin_search_expiry_can_retry_fresh_identical_geometry(self):
        manager, snapshot = self.margin_manager()
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())

        def expire_search(joints):
            self.now.return_value = 100.6
            return None

        with patch.object(manager.planner, 'find_human_margin_retreat',
                          side_effect=expire_search):
            self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
        revision = manager.planner.collision_filter.environment_revision
        manager.update_human_snapshot(snapshot)
        manager.update_joint_state(manager.current_q.copy())
        manager.update_obstacles([])
        self.assertEqual(manager.planner.collision_filter.environment_revision, revision)
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')

    def test_batched_human_distances_match_scalar_for_every_pair(self):
        rng = np.random.default_rng(123)
        for _ in range(20):
            geometries = [(f'g{i}', rng.normal(size=3), rng.uniform(.01, .2)) for i in range(5)]
            capsules = tuple(((1, i), tuple(rng.normal(size=3)), tuple(rng.normal(size=3)),
                              rng.uniform(.03, .2)) for i in range(8))
            # Exercise a degenerate capsule as well as finite segments.
            item = capsules[0]
            capsules = ((item[0], item[1], item[1], item[3]),) + capsules[1:]
            actual = nearest_human_collision(geometries, capsules, .1, include_pairs=True)
            expected = {}
            for i, (_, center, radius) in enumerate(geometries):
                center = FrameTransform.ROBOT_FROM_MUJOCO_BASE @ center
                for key, start, end, human_radius in capsules:
                    distance = point_segment_distance(center, np.array(start), np.array(end))
                    expected[(i, key[0], key[1])] = distance - radius - human_radius
            self.assertEqual(actual['pairs'].keys(), expected.keys())
            for key, value in expected.items():
                self.assertAlmostEqual(actual['pairs'][key], value, places=12)
            closest = min(expected, key=expected.get)
            self.assertEqual(actual['robot_geom'], geometries[closest[0]][0])
            self.assertEqual(actual['segment_id'], closest[2])
            self.assertEqual(actual['status'], 'NEAR' if expected[closest] <= .1 else 'FAR')

    def test_batched_invalid_robot_geometry_remains_unknown(self):
        capsules = (((1, 0), (0, 0, 0), (0, 0, 1), .1),)
        for geometry in [('bad', (0, 0), .1), ('bad', (0, np.nan, 0), .1),
                         ('bad', (0, 0, 0), -.1)]:
            result = nearest_human_collision([geometry], capsules, .1)
            self.assertEqual(result['reason'], 'INVALID_ROBOT_GEOMETRY')

    def test_snapshot_copy_cannot_restart_capture_freshness_timeout(self):
        monitor = HumanCapsuleMonitor()
        message = SimpleNamespace(
            valid=True, header=SimpleNamespace(
                frame_id='base_link', stamp=SimpleNamespace(sec=10, nanosec=0)),
            capsules=[SimpleNamespace(body_id=0, segment_id=0, radius=.1,
                                      start=SimpleNamespace(x=10., y=10., z=10.),
                                      end=SimpleNamespace(x=10., y=10., z=11.))])
        self.assertTrue(monitor.update(message, 10_400_000_000))
        snapshot = monitor.snapshot(10_400_000_000)
        self.assertAlmostEqual(snapshot.expires_at, 100.1)
        manager = self.make_manager(snapshot=snapshot)
        checker = manager.planner.collision_filter
        self.now.return_value = 100.05
        manager.update_human_snapshot(monitor.snapshot(10_450_000_000))
        self.assertTrue(checker.human_data_ready())
        self.now.return_value = 100.11
        self.assertFalse(checker.human_data_ready())

    def test_fresh_empty_after_blocked_retreat_resumes_original_target(self):
        manager, _ = self.margin_manager(clearance=-.01)
        target = manager.target_base.copy()
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
        manager.update_human_snapshot(EMPTY)
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.assertFalse(manager.human_margin_waiting_for_data)
        self.assertEqual(manager.step().action, 'RELEASE')
        manager.update_motion_status('IDLE')
        decision = manager.step()
        self.assertEqual(decision.action, 'EXECUTE')
        self.assertTrue(manager.planner.collision_filter.is_path_valid(decision.path))
        np.testing.assert_array_equal(manager.target_base, target)

    def test_fresh_empty_during_retreat_stops_staging_before_normal_planning(self):
        manager, _, path = self.execute_margin_retreat()
        manager.update_joint_state(np.mean(path, axis=0))
        manager.update_human_snapshot(EMPTY)
        decision = manager.step()
        self.assertEqual((decision.action, decision.status), ('STOP', 'HUMAN_RETREAT_CLEAR'))
        self.assertFalse(manager.human_margin_recovery_active)
        manager.update_motion_status('STOPPED')
        self.assertEqual(manager.step().status, 'WAITING_FOR_FEEDBACK')
        manager.update_joint_state(manager.current_q.copy())
        self.assertEqual(manager.step().status, 'PATH_READY')

    def test_expired_empty_does_not_clear_close_human_stop(self):
        for deadline, now in [(100.05, 100.1), (None, 100.6)]:
            with self.subTest(deadline=deadline):
                self.now.return_value = 100.
                manager, _ = self.margin_manager(clearance=-.01)
                manager.step()
                manager.update_motion_status('STOPPED')
                manager.update_joint_state(manager.current_q.copy())
                manager.step()
                manager.update_human_snapshot(HumanSnapshot(
                    'EMPTY', 'NO_ACTIVE_CAPSULES', (), expires_at=deadline))
                self.now.return_value = now
                manager.update_joint_state(manager.current_q.copy())
                self.assertEqual(manager.step().status, 'HUMAN_RETREAT_DATA_UNKNOWN')
                self.assertTrue(manager.stop_owned)

    def test_invalid_data_after_empty_does_not_clear_close_human_stop(self):
        manager, _ = self.margin_manager(clearance=-.01)
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        manager.step()
        manager.update_human_snapshot(EMPTY)
        manager.update_human_snapshot(HumanSnapshot('UNKNOWN', 'VISION_INVALID', ()))
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_DATA_UNKNOWN')
        self.assertTrue(manager.stop_owned)

    def test_empty_after_retreat_still_requires_static_collision_validation(self):
        manager, _ = self.margin_manager(clearance=-.01)
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        manager.step()
        checker = manager.planner.collision_filter
        checker._set_joint_configuration(manager.current_q)
        center = checker.data.geom_xpos[checker.model.geom('link5_visual').id].copy()
        checker.set_obstacle_pose('obstacle_1', center)
        manager.update_human_snapshot(EMPTY)
        decision = manager.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertNotEqual(checker.contact_report(manager.current_q)['status'], 'CLEAR')
        self.assertTrue(manager.stop_owned)
        self.assertIsNone(manager.current_path)
        self.assertEqual(manager.step().action, 'NONE')

    def test_fresh_empty_does_not_override_unowned_stop(self):
        manager, _ = self.margin_manager()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy())
        manager.step()
        manager.update_human_snapshot(EMPTY)
        self.assertEqual(manager.step().status, 'PATH_READY')
        self.assertEqual(manager.step().status, 'STOPPED')
        self.assertFalse(manager.stop_owned)

    def test_margin_retreat_waits_for_measured_braking_after_stopped_ack(self):
        manager, _ = self.margin_manager()
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy(), [.6, 0., 0., 0., 0.])
        with patch.object(manager.planner, 'find_human_margin_retreat',
                          wraps=manager.planner.find_human_margin_retreat) as search:
            self.assertEqual(manager.step().status, 'HUMAN_RETREAT_BLOCKED')
            self.assertEqual(manager.human_margin_block_reason, 'ROBOT_STILL_MOVING')
            self.assertFalse(search.called)
            manager.update_joint_state(manager.current_q.copy(), np.zeros(5))
            self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')
            self.assertTrue(search.called)

    def test_braking_during_release_cancels_retreat_before_execution(self):
        manager, _ = self.margin_manager()
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(manager.current_q.copy(), np.zeros(5))
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')
        self.assertEqual(manager.step().action, 'RELEASE')
        manager.update_joint_state(manager.current_q.copy(), [.2, 0., 0., 0., 0.])
        decision = manager.step()
        self.assertEqual((decision.action, decision.status), ('STOP', 'WAITING_FOR_ROBOT_SETTLE'))
        self.assertFalse(manager.human_margin_recovery_active)

    def test_invalid_velocity_does_not_refresh_joint_feedback(self):
        manager = self.make_manager()
        received = manager.feedback_time
        self.now.return_value = 100.1
        self.assertFalse(manager.update_joint_state(manager.current_q.copy(), [float('nan')] * 5))
        self.assertEqual(manager.feedback_time, received)

    def test_budget_limited_retreat_can_make_certified_progress_in_stages(self):
        manager, snapshot = self.margin_manager(clearance=.02)
        planner = manager.planner
        checker = planner.collision_filter
        start = manager.current_q.copy()
        target = manager.target_base.copy()
        path = planner.find_human_margin_retreat(start, max_steps=4)
        self.assertIsNotNone(path)
        end_state = checker.human_margin_state(path[-1])
        self.assertEqual(end_state['status'], 'NEAR')
        self.assertGreaterEqual(end_state['clearance'] - .02, checker.safety_margin)
        self.assertTrue(checker.is_human_margin_transition_valid(path[0], path[-1]))
        self.assertFalse(checker.is_path_valid(path))
        with patch.object(planner, 'find_human_margin_retreat', return_value=path):
            self.assertEqual(manager.step().action, 'STOP')
            manager.update_motion_status('STOPPED')
            manager.update_joint_state(start, np.zeros(5))
            self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')
            self.assertEqual(manager.step().action, 'RELEASE')
            manager.update_motion_status('IDLE')
            decision = manager.step()
            self.assertEqual((decision.action, decision.status), ('EXECUTE', 'HUMAN_RETREAT_SENT'))
        manager.update_motion_status('EXECUTING')
        manager.update_joint_state(np.mean(path, axis=0))
        self.assertEqual(manager.step().status, 'EXECUTING')
        manager.update_joint_state(path[-1], np.zeros(5))
        manager.update_motion_status('COMPLETED')
        self.now.return_value = 100.5
        manager.update_human_snapshot(snapshot)
        manager.update_joint_state(path[-1], np.zeros(5))
        manager.update_obstacles([])
        self.assertEqual(manager.step().action, 'STOP')
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(path[-1], np.zeros(5))
        self.assertEqual(manager.step().status, 'HUMAN_RETREAT_READY')
        np.testing.assert_array_equal(manager.target_base, target)

    def test_partial_retreat_cannot_use_expired_geometry(self):
        manager, _ = self.margin_manager(clearance=.02)
        checker = manager.planner.collision_filter
        original = checker.is_human_margin_transition_valid

        def expire_final_segment(start, goal):
            if np.max(np.abs(goal - start)) > .03:
                self.now.return_value = 100.6
            return original(start, goal)

        with patch.object(checker, 'is_human_margin_transition_valid',
                          side_effect=expire_final_segment):
            self.assertIsNone(manager.planner.find_human_margin_retreat(
                manager.current_q.copy(), max_steps=4))

    def test_retreat_screening_skips_one_invalid_candidate_and_keeps_searching(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter
        original = checker.human_margin_state
        blocked = manager.current_q.copy()
        blocked[0] -= .01

        def invalid_candidate(joints):
            return None if np.array_equal(joints, blocked) else original(joints)

        with patch.object(checker, 'human_margin_state', side_effect=invalid_candidate):
            path = manager.planner.find_human_margin_retreat(manager.current_q.copy())
        self.assertIsNotNone(path)
        self.assertTrue(checker.is_human_margin_transition_valid(path[0], path[-1]))
        self.assertTrue(checker.is_state_valid(path[-1]))

    def test_improving_endpoints_cannot_bypass_full_retreat_certificate(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter
        with patch.object(checker, 'is_human_margin_transition_valid', return_value=False):
            self.assertIsNone(manager.planner.find_human_margin_retreat(manager.current_q.copy()))

    def test_expiry_during_retreat_candidate_screening_stays_blocked(self):
        manager, _ = self.margin_manager()
        checker = manager.planner.collision_filter
        original = checker.human_margin_state
        count = 0

        def expire_after_initial(joints):
            nonlocal count
            count += 1
            if count > 1:
                self.now.return_value = 100.6
            return original(joints)

        with patch.object(checker, 'human_margin_state', side_effect=expire_after_initial):
            self.assertIsNone(manager.planner.find_human_margin_retreat(manager.current_q.copy()))
        self.assertEqual(manager.planner.last_human_retreat_failure, 'HUMAN_DATA_UNKNOWN')

    def disconnected_goal_manager(self):
        """Use real robot states with two nearby Cartesian goals on separate branches."""
        manager = self.make_manager()
        planner = manager.planner
        checker = planner.collision_filter
        first = planner.select_goal(manager.target_base, .12, .04)
        second, start = None, None
        for candidate in planner.goal_candidates(manager.target_base, .12, .04):
            if np.max(np.abs(candidate - first)) <= 1.:
                continue
            probe = candidate.copy()
            probe[0] += .03
            if checker.is_state_valid(probe) and checker.is_edge_valid(probe, candidate):
                second, start = candidate, probe
                break
        self.assertIsNotNone(second)
        indices = [int(np.flatnonzero(np.all(planner.joint_q == goal, axis=1))[0])
                   for goal in (first, second)]
        planner.joint_q = np.vstack((first, second))
        planner.joint_eof = planner.joint_eof[indices].copy()
        planner.joint_edges = np.empty((0, 2), dtype=np.int64)
        planner.joint_costs = np.empty(0)
        planner.planner = type(planner.planner)(
            planner.joint_q, planner.joint_edges, planner.joint_costs, checker,
            max_joint_delta=planner.max_joint_delta, connection_count=1)
        manager.update_joint_state(start, np.zeros(5))
        return manager, first, second

    def test_other_joint_goal_for_same_cartesian_target_can_reach_motion(self):
        manager, first, second = self.disconnected_goal_manager()
        target = manager.target_base.copy()
        self.assertEqual(manager.step().status, 'PATH_READY')
        np.testing.assert_array_equal(manager.goal_q, second)
        self.assertFalse(np.array_equal(manager.goal_q, first))
        self.assertEqual(manager.step().action, 'EXECUTE')
        self.assertTrue(manager.planner.collision_filter.human_obstacles_enabled)
        self.assertTrue(manager.planner.is_path_valid(manager.last_execution_path))
        np.testing.assert_array_equal(manager.target_base, target)

    def test_goal_candidate_limit_does_not_turn_failure_into_unchecked_motion(self):
        manager, _, _ = self.disconnected_goal_manager()
        manager.planner.max_goal_candidates = 1
        self.assertEqual(manager.step().status, 'NO_PATH')
        self.assertIsNone(manager.current_path)
        self.assertEqual(manager.step().action, 'NONE')

    def test_unsafe_or_expired_input_does_not_trigger_goal_alternatives(self):
        manager = self.make_manager()
        planner = manager.planner
        goal = planner.select_goal(manager.target_base, .12, .04)
        for reason in ('START_INVALID', 'GOAL_INVALID', 'HUMAN_VISION_UNKNOWN', 'PATH_INVALID'):
            planner.last_plan_failure = reason
            with patch.object(planner, 'plan') as plan:
                result = planner.find_alternative_goal_path(
                    manager.current_q, manager.target_base, goal, .12, .04)
                self.assertIsNone(result)
                self.assertFalse(plan.called)

    def test_goal_alternatives_stop_when_observation_expires(self):
        manager = self.make_manager()
        planner = manager.planner
        goal = planner.select_goal(manager.target_base, .12, .04)
        self.now.return_value = 100.6
        planner.last_plan_failure = 'NO_GRAPH_PATH'
        with patch.object(planner, 'plan') as plan:
            self.assertIsNone(planner.find_alternative_goal_path(
                manager.current_q, manager.target_base, goal, .12, .04))
            self.assertFalse(plan.called)
        self.assertEqual(planner.last_plan_failure, 'HUMAN_VISION_UNKNOWN')

    def test_first_failed_goal_batch_can_stage_then_replan_original_target_once(self):
        manager = self.make_manager(EMPTY, joints=START)
        target = manager.target_base.copy()
        checker = manager.planner.collision_filter
        with patch.object(manager.planner, 'plan', return_value=False):
            manager.planner.last_plan_failure = 'NO_GRAPH_PATH'
            self.assertEqual(manager.step().status, 'SAFE_POSITION_READY')
            path = manager.current_path.copy()
            self.assertTrue(checker.is_path_valid(path))
            self.assertEqual(manager.step().action, 'EXECUTE')
            manager.update_motion_status('EXECUTING')
            manager.update_joint_state(path[-1], np.zeros(5))
            manager.update_motion_status('COMPLETED')
            self.assertTrue(manager.planning_required)
            self.assertFalse(manager.safe_position_active)
            self.assertEqual(manager.step().status, 'NO_PATH')
            self.assertIsNone(manager.current_path)
            self.assertEqual(manager.step().action, 'NONE')
        np.testing.assert_array_equal(manager.target_base, target)

    def test_staging_does_not_override_an_unowned_stop(self):
        manager = self.make_manager(EMPTY, joints=START)
        manager.update_motion_status('STOPPED')
        manager.update_joint_state(START, np.zeros(5))
        with patch.object(manager.planner, 'plan', return_value=False):
            manager.planner.last_plan_failure = 'NO_GRAPH_PATH'
            self.assertEqual(manager.step().status, 'SAFE_POSITION_READY')
            decision = manager.step()
            self.assertEqual((decision.action, decision.status), ('NONE', 'STOPPED'))
            self.assertFalse(manager.stop_owned)

    def test_invalid_planned_path_does_not_trigger_generic_staging(self):
        manager = self.make_manager(EMPTY, joints=START)
        manager.human_retry_count = 1
        with patch.object(manager.planner, 'plan', return_value=False), \
                patch.object(manager, '_prepare_safe_position') as stage:
            manager.planner.last_plan_failure = 'PATH_INVALID'
            self.assertEqual(manager.step().status, 'NO_PATH')
            self.assertFalse(stage.called)

    def test_invalidating_a_staging_move_clears_its_completion_context(self):
        manager = self.make_manager(EMPTY, joints=START)
        manager.safe_position_active = True
        manager.invalidate_path()
        self.assertFalse(manager.safe_position_active)

    def test_staging_expiry_can_retry_after_inputs_are_refreshed(self):
        for expired_input in ('human', 'feedback', 'obstacles'):
            with self.subTest(expired_input=expired_input):
                manager = self.make_manager(FAR, joints=START)
                checker = manager.planner.collision_filter
                validate = checker.is_path_valid

                def expire_after_validation(path):
                    result = validate(path)
                    if expired_input == 'human':
                        self.now.return_value = 100.6
                    elif expired_input == 'feedback':
                        manager.feedback_time -= manager.feedback_timeout + .1
                    else:
                        manager.obstacle_time -= manager.obstacle_timeout + .1
                    return result

                with patch.object(checker, 'is_path_valid', side_effect=expire_after_validation):
                    self.assertFalse(manager._prepare_safe_position())
                self.assertIsNone(manager.safe_position_attempted_revision)
                self.assertFalse(manager.safe_position_active)
                self.now.return_value = 100.
                manager.update_joint_state(START, np.zeros(5))
                manager.update_obstacles([])
                manager.update_human_snapshot(FAR)
                self.assertTrue(manager._prepare_safe_position())

    def test_tracking_fault_cannot_be_released_by_empty_vision_or_staging(self):
        manager = self.make_manager(EMPTY, joints=START)
        manager.recovery_attempted = True
        manager.update_failure_reason('TRACKING_ERROR')
        manager.update_motion_status('CONTROL_ERROR')
        self.assertTrue(manager.fault)
        manager.update_human_snapshot(EMPTY)
        with patch.object(manager, '_prepare_safe_position') as stage, \
                patch.object(manager.planner, 'plan') as plan:
            decision = manager.step()
            self.assertEqual((decision.action, decision.status), ('NONE', 'MOTION_ERROR'))
            self.assertFalse(stage.called)
            self.assertFalse(plan.called)
        self.assertTrue(manager.fault)


if __name__ == '__main__':
    unittest.main(verbosity=2)
