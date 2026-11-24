"""Fallback ownership, stale-result rejection and existing execution safety gates."""

import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from planning.path_planner import PathPlanner
from planning.planning_manager import PlanningManager
from planning.rrt_fallback import RRTFallbackCoordinator, fallback_scene, rrt_settings
from planning.rrt_shadow import capture_scene, run_job
from test_rrt_shadow import ManualExecutor, options


START = np.zeros(5)
GOAL = np.array([.9, 0., 0., 0., 0.])
PREPARED = np.array([START, GOAL / 2., GOAL])


class Checker:
    def __init__(self):
        self.lower, self.upper = np.full(5, -2.), np.full(5, 2.)
        self.environment_revision = 1
        self.human_obstacles_enabled = True
        self.human_status = 'OBSERVED'
        self.human_received_at, self.human_expires_at = 100., 102.
        self.human_data_timeout = 2.
        self.human_safety_margin, self.max_joint_step, self.safety_margin = .05, .02, .005
        self.human_report_cache_size = 0
        self.human_capsules = (((0, 0), (10., 10., 10.), (10., 10., 11.), .1),)
        self.model = SimpleNamespace(geom_size=np.empty((0, 3)), geom_contype=np.empty(0),
                                     geom_conaffinity=np.empty(0))
        self.data = SimpleNamespace(mocap_pos=np.empty((0, 3)), mocap_quat=np.empty((0, 4)))
        self.last_path_validation_failure = None
        self.last_state_validation_failure = None
        self.last_edge_validation_failure = None
        self.calls = 0

    def enable_human_obstacles(self, enabled):
        if self.human_obstacles_enabled != enabled:
            self.environment_revision += 1
        self.human_obstacles_enabled = enabled

    def human_data_ready(self):
        return (self.human_status == 'OBSERVED' and self.human_received_at is not None
                and time.monotonic() < self.human_expires_at)

    def human_collision_report(self, _):
        return dict(status='FAR' if self.human_data_ready() else 'UNKNOWN',
                    clearance=1., reason='NO_DATA')

    def is_state_valid(self, q):
        self.calls += 1
        return (np.isfinite(q).all() and np.all(q >= self.lower) and np.all(q <= self.upper)
                and (not self.human_obstacles_enabled or self.human_data_ready()))

    def is_path_valid(self, path):
        return all(self.is_state_valid(q) for q in path)


class Planner:
    def __init__(self):
        self.collision_filter = Checker()
        self.current_path = self.goal_q = None
        self.last_plan_failure = 'NO_GRAPH_PATH'
        self.max_segment_delta = .6
        self.joint_goal_tolerance = .02
        self.segment_progress_enabled = False
        self.execution_progress_config = dict(corridor_tolerance=.04)
        self.primary_success = False

    def select_goal(self, *_):
        return GOAL.copy()

    def plan(self, *_):
        if self.primary_success:
            self.current_path = PREPARED.copy()
            self.last_plan_failure = 'NONE'
        return self.primary_success

    def find_alternative_goal_path(self, *_):
        return None

    def human_data_ready(self):
        return (not self.collision_filter.human_obstacles_enabled
                or self.collision_filter.human_data_ready())

    def is_path_valid(self, path):
        return self.collision_filter.is_path_valid(path)

    prepare_execution_path = PathPlanner.prepare_execution_path


class FallbackFixture(unittest.TestCase):
    def setUp(self):
        self.now = [100.]
        self.clock = patch('time.monotonic', side_effect=lambda: self.now[0])
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.p = Planner()
        self.m = PlanningManager(self.p, execute_path=True, human_avoidance_enabled=True,
                                 simulation_only_ignore_unknown=True)
        self.m.rrt_fallback_enabled = True
        self.m.current_q = START.copy()
        self.m.feedback_time = self.m.obstacle_time = 100.
        self.m.obstacles_ready = True
        self.m.target_base = np.array([.2, .1, .3])
        self.m.target_generation = 1
        self.m.human_status = 'OBSERVED'
        self.m.motion_status = 'IDLE'
        safe_patch = patch.object(self.m, '_prepare_safe_position', return_value=False)
        self.safe = safe_patch.start()
        self.addCleanup(safe_patch.stop)

    def request(self):
        self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.assertTrue(self.m.rrt_fallback_pending)
        self.safe.assert_not_called()

    def result(self, **changes):
        result = dict(sequence=self.m.graph_attempt_sequence, target_generation=1, reason='NONE',
                      scene_revision=self.p.collision_filter.environment_revision, expires_at=102.,
                      prepared_path=PREPARED.copy(), start=START.copy())
        result.update(changes)
        return result

    def adopt(self):
        self.request()
        self.assertTrue(self.m.offer_rrt_result(self.result()))
        self.assertIsNone(self.m.current_path)
        self.assertEqual(self.m.step().status, 'RRT_PATH_READY')
        self.assertEqual(self.m.path_source, 'RRT_CONNECT')


class ManagerFallbackTests(FallbackFixture):
    def test_primary_path_wins_without_fallback(self):
        self.p.primary_success = True
        self.assertEqual(self.m.step().status, 'PATH_READY')
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertEqual(self.m.path_source, 'GNG_ASTAR')

    def test_no_graph_path_waits_before_existing_safe_position(self):
        self.request()
        for _ in range(5):
            self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.safe.assert_not_called()

    def test_disabled_fallback_preserves_original_no_path(self):
        self.m.rrt_fallback_enabled = False
        self.assertEqual(self.m.step().status, 'NO_PATH')
        self.safe.assert_called_once()

    def test_invalid_start_does_not_request_rrt(self):
        self.p.last_plan_failure = 'START_INVALID'
        with patch.object(self.m, '_prepare_safe_recovery', return_value=False):
            self.assertEqual(self.m.step().status, 'RECOVERY_BLOCKED')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_unreachable_goal_does_not_request_rrt(self):
        with patch.object(self.p, 'select_goal', return_value=None):
            self.assertEqual(self.m.step().status, 'GOAL_UNREACHABLE')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_unknown_does_not_borrow_simulation_override(self):
        self.m.human_status = 'UNKNOWN'
        self.p.collision_filter.human_status = 'UNKNOWN'
        self.m.step()
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertTrue(self.m.simulation_only_ignore_unknown)

    def test_fresh_empty_can_request_static_scene_fallback(self):
        self.m.human_status = 'EMPTY'
        self.m.human_empty_until = 102.
        self.p.collision_filter.human_status = 'UNKNOWN'
        self.p.collision_filter.human_capsules = ()
        self.request()
        scene = fallback_scene(self.m)
        self.assertEqual(scene['human_data_kind'], 'EMPTY')
        self.assertFalse(scene['human_obstacles_enabled'])
        self.assertEqual(scene['human_valid_until'], 102.)

    def test_manual_stop_is_never_released_by_fallback_request(self):
        self.m.motion_status = 'STOPPED'
        self.m.step()
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertFalse(self.m.stop_owned)

    def test_mailbox_is_not_execution_and_cannot_be_overwritten(self):
        self.request()
        self.assertTrue(self.m.offer_rrt_result(self.result()))
        self.assertFalse(self.m.offer_rrt_result(self.result()))
        self.assertIsNone(self.m.current_path)
        self.assertIsNone(self.m.pending_action)

    def test_execution_uses_existing_preparation_and_final_validation(self):
        self.adopt()
        before = self.p.collision_filter.calls
        with patch.object(self.p, 'prepare_execution_path',
                          wraps=self.p.prepare_execution_path) as prepare:
            decision = self.m.step()
        self.assertEqual(decision.action, 'EXECUTE')
        prepare.assert_called_once()
        self.assertGreater(self.p.collision_filter.calls, before)
        np.testing.assert_array_equal(decision.path[-1], GOAL)

    def test_stopped_robot_requires_release_ack_then_revalidation(self):
        self.m.motion_status, self.m.stop_owned = 'STOPPED', True
        self.adopt()
        self.assertEqual(self.m.step().action, 'RELEASE')
        self.assertEqual(self.m.step().status, 'WAITING_FOR_RELEASE')
        self.m.update_motion_status('IDLE')
        self.assertEqual(self.m.step().action, 'EXECUTE')

    def test_expired_result_is_rejected_without_motion(self):
        self.request()
        self.m.offer_rrt_result(self.result(expires_at=99.))
        self.assertEqual(self.m.step().status, 'RRT_FAILED')
        self.assertEqual(self.m.rrt_last_reason, 'VALIDATION_SNAPSHOT_EXPIRED')
        self.assertIsNone(self.m.current_path)

    def test_changed_validation_scene_restarts_primary(self):
        self.request()
        self.m.offer_rrt_result(self.result(scene_revision=-1))
        self.assertEqual(self.m.step().status, 'RRT_REPLANNING')
        self.assertTrue(self.m.planning_required)

    def test_old_target_or_attempt_cannot_fill_mailbox(self):
        self.request()
        self.assertFalse(self.m.offer_rrt_result(self.result(target_generation=2)))
        self.assertFalse(self.m.offer_rrt_result(self.result(sequence=0)))
        self.m.invalidate_path()
        self.assertFalse(self.m.offer_rrt_result(self.result()))

    def test_invalid_worker_paths_fail_closed(self):
        for path in (np.zeros(5), PREPARED * np.nan, PREPARED[::-1],
                     np.array([START, [5., 0., 0., 0., 0.], GOAL])):
            with self.subTest(path=path):
                self.m.planning_required = True
                self.m.human_replan_blocked = False
                self.m.rrt_fallback_pending = False
                self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
                self.m.offer_rrt_result(self.result(prepared_path=path))
                self.assertEqual(self.m.step().status, 'RRT_FAILED')
                self.assertEqual(self.m.rrt_last_reason, 'INVALID_WORKER_PATH')

    def test_malformed_certificate_does_not_crash_or_execute(self):
        self.request()
        self.m.offer_rrt_result(self.result(expires_at='invalid'))
        self.assertEqual(self.m.step().status, 'RRT_FAILED')
        self.assertIsNone(self.m.current_path)

    def test_timeout_and_worker_failure_fall_back_to_existing_recovery(self):
        self.request()
        self.m.offer_rrt_result(self.result(reason='TIME_BUDGET_EXHAUSTED'))
        self.assertEqual(self.m.step().status, 'RRT_FAILED')
        self.safe.assert_called_once()

    def test_manager_timeout_does_not_need_worker_completion(self):
        self.request()
        self.now[0] = 101.3
        self.m.feedback_time = self.m.obstacle_time = self.now[0]
        self.assertEqual(self.m.step().status, 'RRT_FAILED')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_start_movement_rejects_worker_certificate(self):
        self.request()
        self.m.current_q[0] = .02
        self.m.offer_rrt_result(self.result())
        self.assertEqual(self.m.step().status, 'RRT_REPLANNING')

    def test_camera_loss_before_dispatch_never_executes(self):
        self.adopt()
        self.m.human_status = self.p.collision_filter.human_status = 'UNKNOWN'
        decision = self.m.step()
        self.assertEqual(decision.status, 'RRT_HUMAN_VISION_UNKNOWN')
        self.assertEqual(decision.action, 'NONE')

    def test_camera_loss_during_execution_requests_stop(self):
        self.adopt()
        self.assertEqual(self.m.step().action, 'EXECUTE')
        self.m.update_motion_status('EXECUTING')
        self.m.human_status = self.p.collision_filter.human_status = 'UNKNOWN'
        self.assertEqual(self.m.step().action, 'STOP')

    def test_close_human_cancels_pending_fallback_and_uses_existing_stop(self):
        self.request()
        self.m.human_margin_recovery_enabled = True
        with patch.object(self.p.collision_filter, 'human_collision_report',
                          return_value=dict(status='NEAR', clearance=.01)):
            self.assertEqual(self.m.step().action, 'STOP')
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertTrue(self.m.stop_owned)


class InputRefreshTests(FallbackFixture):
    def defer(self, *, feedback=True, obstacles=True):
        def failed_search(*_):
            self.now[0] += 1.21
            if not feedback:
                self.m.feedback_time = self.now[0]
            if not obstacles:
                self.m.obstacle_time = self.now[0]
            return False

        with patch.object(self.p, 'plan', side_effect=failed_search) as search:
            decision = self.m.step()
        self.assertEqual(search.call_count, 1)
        self.assertIn(decision.status, ('RRT_WAITING_FOR_FEEDBACK', 'RRT_WAITING_FOR_OBSTACLES'))
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertIsNotNone(self.m.rrt_refresh_attempt)
        self.safe.assert_not_called()
        return self.m.rrt_refresh_attempt['deadline']

    def refresh(self):
        self.m.update_joint_state(START)
        self.m.obstacle_time = self.now[0]

    def test_long_primary_search_yields_then_requests_rrt_without_repeating_astar(self):
        deadline = self.defer()
        sequence = self.m.graph_attempt_sequence
        self.now[0] += .05
        self.refresh()
        with patch.object(self.p, 'plan', side_effect=AssertionError('Repeated A*')):
            self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.assertEqual(self.m.graph_attempt_sequence, sequence)
        self.assertEqual(self.m.rrt_fallback_attempt['deadline'], deadline)
        self.assertIsNone(self.m.rrt_refresh_attempt)
        self.assertIsNone(self.m.pending_action)

    def test_unchanged_stale_input_waits_without_astar_or_deadline_extension(self):
        deadline = self.defer()
        for _ in range(3):
            self.now[0] += .05
            with patch.object(self.p, 'plan', side_effect=AssertionError('Repeated A*')):
                self.assertEqual(self.m.step().status, 'RRT_WAITING_FOR_FEEDBACK')
            self.assertEqual(self.m.rrt_refresh_attempt['deadline'], deadline)

    def test_feedback_then_obstacle_callbacks_are_both_required(self):
        self.defer()
        self.m.update_joint_state(START)
        self.assertEqual(self.m.step().status, 'RRT_WAITING_FOR_OBSTACLES')
        self.m.obstacle_time = self.now[0]
        self.assertEqual(self.m.step().status, 'RRT_SEARCHING')

    def test_obstacle_only_staleness_can_defer(self):
        self.defer(feedback=False)
        self.assertEqual(self.m.rrt_block_reason, 'OBSTACLES_STALE')

    def test_pending_refresh_expires_and_never_executes(self):
        deadline = self.defer()
        self.now[0] = deadline + .01
        decision = self.m.step()
        self.assertIsNone(self.m.rrt_refresh_attempt)
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_HANDOFF_EXPIRED')

    def test_target_change_invalidates_pending_refresh(self):
        self.defer()
        self.m.target_generation += 1
        self.refresh()
        decision = self.m.step()
        self.assertEqual(decision.status, 'RRT_REPLANNING')
        self.assertIsNone(self.m.rrt_refresh_attempt)
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_TARGET_CHANGED')

    def test_moved_start_invalidates_pending_refresh(self):
        self.defer()
        self.refresh()
        self.m.current_q[0] += .02
        self.assertEqual(self.m.step().status, 'RRT_REPLANNING')
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_START_MOVED')

    def test_new_human_snapshot_requires_endpoint_revalidation(self):
        self.defer()
        self.refresh()
        self.p.collision_filter.environment_revision += 1
        with patch.object(self.p.collision_filter, 'is_state_valid',
                          side_effect=lambda q: not np.array_equal(q, GOAL)):
            decision = self.m.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_GOAL_INVALID')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_human_too_close_cancels_refresh_and_retreat_keeps_priority(self):
        self.defer()
        self.refresh()
        self.m.motion_status, self.m.stop_owned = 'STOPPED', True
        self.m.human_margin_recovery_enabled = True
        with patch.object(self.p.collision_filter, 'human_collision_report',
                          return_value=dict(status='NEAR', clearance=.03)):
            with patch.object(self.m, '_prepare_human_margin_retreat', return_value=False):
                decision = self.m.step()
        self.assertEqual(decision.status, 'HUMAN_RETREAT_BLOCKED')
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_HUMAN_TOO_CLOSE')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_unknown_human_data_does_not_borrow_simulation_override(self):
        self.defer()
        self.refresh()
        self.m.human_status = self.p.collision_filter.human_status = 'UNKNOWN'
        self.m.step()
        self.assertIsNone(self.m.rrt_refresh_attempt)
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_HUMAN_VISION_UNKNOWN')

    def test_static_scene_invalidation_cancels_refresh(self):
        self.defer()
        self.m.invalidate_path()
        self.assertIsNone(self.m.rrt_refresh_attempt)
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_external_stop_cannot_be_released_by_waiting_attempt(self):
        self.defer()
        self.refresh()
        self.m.motion_status = 'STOPPED'
        self.m.stop_owned = False
        decision = self.m.step()
        self.assertEqual(decision.action, 'NONE')
        self.assertEqual(self.m.rrt_last_reason, 'CANCELLED_EXTERNAL_STOP')


class CoordinatorTests(FallbackFixture):
    def setUp(self):
        super().setUp()
        self.executor = ManualExecutor()
        self.coordinator = RRTFallbackCoordinator(
            'unused', ['j'] * 5, options(self.p.collision_filter),
            executor=self.executor, fallback_budget=1.2)
        self.addCleanup(self.coordinator.close)

    def start(self):
        self.request()
        self.assertEqual(self.coordinator.tick(self.m)['event'], 'SEARCH_SUBMITTED')

    def validation(self):
        self.start()
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='NONE', path=np.array([START, GOAL])))
        self.assertEqual(self.coordinator.tick(self.m)['event'], 'LATEST_VALIDATION_SUBMITTED')

    def test_coordinator_handoff_is_prepared_and_not_a_command(self):
        self.validation()
        self.executor.jobs[-1][2].set_result(dict(
            kind='validate', reason='NONE', prepared_path=PREPARED))
        self.assertEqual(self.coordinator.tick(self.m)['event'], 'FALLBACK_VALIDATED')
        self.assertIsNone(self.m.current_path)
        self.m.offer_rrt_result(self.coordinator.delivery)
        self.assertEqual(self.m.step().status, 'RRT_PATH_READY')
        self.assertIsNone(self.m.pending_action)

    def test_coordinator_search_does_not_require_rviz_or_preference(self):
        self.start()
        job = self.executor.jobs[-1][1]
        self.assertFalse(job['visualize'])
        self.assertFalse(job['use_preference'])
        self.assertLessEqual(job['request_deadline'], self.m.rrt_fallback_attempt['deadline'])

    def test_coordinator_validates_on_latest_scene(self):
        self.start()
        self.p.collision_filter.environment_revision += 1
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='NONE', path=np.array([START, GOAL])))
        self.coordinator.tick(self.m)
        self.assertEqual(self.executor.jobs[-1][1]['scene']['revision'],
                         self.p.collision_filter.environment_revision)

    def test_coordinator_scene_changes_during_validation_reject(self):
        self.validation()
        self.p.collision_filter.environment_revision += 1
        self.executor.jobs[-1][2].set_result(dict(
            kind='validate', reason='NONE', prepared_path=PREPARED))
        self.assertEqual(self.coordinator.tick(self.m)['reason'], 'ENVIRONMENT_CHANGED')

    def test_coordinator_late_running_job_cannot_revive_request(self):
        self.start()
        self.now[0] = 101.3
        self.m.feedback_time = self.m.obstacle_time = self.now[0]
        self.assertEqual(self.coordinator.tick(self.m)['reason'], 'TIME_BUDGET_EXHAUSTED')
        self.m.offer_rrt_result(self.coordinator.delivery)
        self.m.step()
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='NONE', path=np.array([START, GOAL])))
        self.assertEqual(self.coordinator.tick(self.m)['event'], 'DISCARDED')
        self.assertIsNone(self.m.current_path)

    def test_coordinator_worker_error_fails_closed(self):
        self.start()
        self.executor.jobs[-1][2].set_exception(RuntimeError('worker failed'))
        self.assertEqual(self.coordinator.tick(self.m)['reason'], 'WORKER_ERROR')
        self.assertTrue(self.coordinator.disabled)

    def test_coordinator_target_change_discards_without_handoff(self):
        self.start()
        self.m.invalidate_path()
        self.m.target_generation += 1
        self.assertEqual(self.coordinator.tick(self.m)['event'], 'FALLBACK_REJECTED')
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='NONE', path=np.array([START, GOAL])))
        self.assertEqual(self.coordinator.tick(self.m)['event'], 'DISCARDED')
        self.assertEqual(len(self.executor.jobs), 1)

    def test_coordinator_handoff_never_borrows_time_from_new_observations(self):
        self.p.collision_filter.human_expires_at = 100.8
        self.start()
        self.p.collision_filter.human_expires_at = 102.
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='NONE', path=np.array([START, GOAL])))
        self.coordinator.tick(self.m)
        self.executor.jobs[-1][2].set_result(dict(
            kind='validate', reason='NONE', prepared_path=PREPARED))
        self.coordinator.tick(self.m)
        self.assertEqual(self.coordinator.delivery['expires_at'], 100.8)
        self.now[0] = 100.9
        self.m.offer_rrt_result(self.coordinator.delivery)
        self.assertEqual(self.m.step().status, 'RRT_FAILED')
        self.assertIsNone(self.m.current_path)


class NativeWorkerTests(unittest.TestCase):
    def setUp(self):
        from test_rrt_visualization import native_planner
        self.clock = patch('time.monotonic', return_value=100.)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.p = native_planner()

    def test_real_worker_returns_prepared_baseline_without_clearance_selection(self):
        from benchmark_rrt_connect import START as a, GOAL as b
        checker = self.p.collision_filter
        scene = capture_scene(checker)
        job = dict(kind='search', scene=scene, sample_step=.01, search_budget=1.,
                   start=a, goal=b, seed=0, total_budget=1., request_deadline=101.5,
                   use_preference=False)
        with patch('planning.rrt_shadow.ClearancePathPreference',
                   side_effect=AssertionError('Unexpected extra selection')):
            result = run_job(job, checker)
        self.assertEqual(result['reason'], 'NONE')
        prepared = run_job(dict(
            kind='validate', scene=scene, sample_step=.01, start=a, path=result['path'],
            max_segment_delta=.6, total_budget=1., request_deadline=101.5,
            return_prepared=True), checker)
        self.assertEqual(prepared['reason'], 'NONE')
        self.assertTrue(checker.is_path_valid(prepared['prepared_path']))
        self.assertLessEqual(np.max(np.abs(np.diff(prepared['prepared_path'], axis=0))), .6)

    def test_expired_empty_cannot_become_blind_static_scene(self):
        scene = capture_scene(self.p.collision_filter)
        scene.update(human_data_kind='EMPTY', human_valid_until=99.,
                     human_obstacles_enabled=False, human_capsules=())
        result = run_job(dict(kind='search', scene=scene, sample_step=.01),
                         self.p.collision_filter)
        self.assertEqual(result['reason'], 'HUMAN_VISION_UNKNOWN')
        self.assertIsNone(result['path'])

    def test_fresh_empty_can_be_certified_with_real_static_collision_checks(self):
        from benchmark_rrt_connect import START as a, GOAL as b
        checker = self.p.collision_filter
        scene = capture_scene(checker)
        scene.update(human_data_kind='EMPTY', human_valid_until=102.,
                     human_obstacles_enabled=False, human_capsules=(), human_status='UNKNOWN',
                     human_received_at=None, human_expires_at=None)
        result = run_job(dict(kind='search', scene=scene, sample_step=.01, search_budget=1.,
                              start=a, goal=b, seed=0, total_budget=1., request_deadline=101.5,
                              use_preference=False), checker)
        self.assertEqual(result['reason'], 'NONE')
        self.assertTrue(checker.is_path_valid(result['path']))


class ModeTests(unittest.TestCase):
    def test_fallback_is_rejected_for_physical_robot_before_node_creation(self):
        with self.assertRaisesRegex(ValueError, 'simulation-only'):
            rrt_settings(dict(rrt=dict(mode='fallback')), 'robot')

    def test_shadow_mode_retains_legacy_configuration(self):
        mode, visible, options_ = rrt_settings(dict(rrt_shadow=dict(enabled=True)), 'simulation')
        self.assertEqual(mode, 'shadow')
        self.assertTrue(visible)
        self.assertNotIn('fallback_budget', options_)

    def test_explicit_off_can_override_yaml_fallback(self):
        mode, _, _ = rrt_settings(dict(rrt=dict(mode='fallback')), 'robot', 'off')
        self.assertEqual(mode, 'off')

    def test_invalid_modes_fail_closed(self):
        for mode in ('rrt_only', True):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                rrt_settings(dict(rrt=dict(mode=mode)), 'simulation')

    def test_invalid_fallback_budget_does_not_create_worker(self):
        for budget in (0., True, float('inf')):
            with self.subTest(budget=budget), self.assertRaises(ValueError):
                RRTFallbackCoordinator('unused', ['j'] * 5, options(Checker()),
                                       fallback_budget=budget)


if __name__ == '__main__':
    unittest.main()
