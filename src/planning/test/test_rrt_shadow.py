"""Observation jobs cannot alter commands or revive expired scene snapshots."""

from concurrent.futures import Future
from dataclasses import replace
import os
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from benchmark_rrt_connect import FAR, GOAL, MODEL, START, make_scenes, planner
from planning.rrt_shadow import ShadowRRTObserver, capture_scene, run_job


class ManualExecutor:
    def __init__(self):
        self.jobs = []
        self.closed = False

    def submit(self, function, job):
        future = Future()
        self.jobs.append((function, job, future))
        return future

    def shutdown(self, **kwargs):
        self.closed = True


def options(checker):
    return {key: getattr(checker, key) for key in (
        'safety_margin', 'max_joint_step', 'human_safety_margin',
        'human_data_timeout', 'human_report_cache_size')}


def manager_for(p):
    return SimpleNamespace(
        target_base=np.array([.1, .2, .3]), target_generation=1,
        last_graph_failure=dict(sequence=1, target_generation=1,
                                start=START.copy(), goal=GOAL.copy()),
        current_q=START.copy(), fault=False, motion_active=False,
        pending_action=None, motion_status='STOPPED', safe_recovery_active=False,
        safe_position_active=False, human_margin_recovery_active=False,
        feedback_ready=lambda: True, obstacle_data_ready=lambda: True,
        feedback_time=time.monotonic(), planner=p)


class ShadowCoordinatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # These tests exercise scheduling, not asset loading. Share immutable
        # model geometry; native tests below separately cover actual isolation.
        cls.shared_planner = planner(FAR)

    def setUp(self):
        self.p = self.shared_planner
        self.p.collision_filter.enable_human_obstacles(True)
        self.p.collision_filter.set_human_capsules(FAR)
        self.manager = manager_for(self.p)
        self.executor = ManualExecutor()
        self.observer = ShadowRRTObserver(
            MODEL, self.p.joint_names, options(self.p.collision_filter), executor=self.executor)
        self.addCleanup(self.observer.close)

    def submit(self):
        self.assertEqual(self.observer.tick(self.manager)['event'], 'SEARCH_SUBMITTED')

    def finish_search(self):
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='NONE', path=np.array([START, GOAL]), elapsed_s=.1))

    def submit_validation(self):
        self.submit()
        self.finish_search()
        self.assertEqual(self.observer.tick(self.manager)['event'],
                         'LATEST_VALIDATION_SUBMITTED')

    def finish_validation(self):
        self.executor.jobs[-1][2].set_result(dict(
            kind='validate', reason='NONE', path=None, elapsed_s=.1, points=50))

    def test_poll_is_nonblocking_and_queue_has_only_one_job(self):
        self.submit()
        for _ in range(30):
            self.assertIsNone(self.observer.tick(self.manager))
        self.assertEqual(len(self.executor.jobs), 1)
        self.assertFalse(self.observer.future.done())

    def test_finished_candidate_is_revalidated_on_new_human_snapshot(self):
        self.submit()
        checker = self.p.collision_filter
        checker.set_human_capsules(replace(FAR, expires_at=time.monotonic() + 1.))
        self.finish_search()
        self.observer.tick(self.manager)
        self.assertEqual(self.executor.jobs[-1][1]['kind'], 'validate')
        self.assertEqual(self.executor.jobs[-1][1]['scene']['human_expires_at'],
                         checker.human_expires_at)

    def test_new_human_geometry_after_validation_never_gets_current_label(self):
        self.submit_validation()
        self.p.collision_filter.environment_revision += 1
        self.finish_validation()
        result = self.observer.tick(self.manager)
        self.assertEqual(result['disposition'], 'ENVIRONMENT_CHANGED_AFTER_VALIDATION')

    def test_target_change_is_latched_even_when_target_changes_back(self):
        self.submit()
        self.manager.target_generation += 1
        self.observer.tick(self.manager)
        self.manager.target_generation -= 1
        self.finish_search()
        result = self.observer.tick(self.manager)
        self.assertEqual(result['disposition'], 'TARGET_CHANGED')
        self.assertEqual(len(self.executor.jobs), 1)

    def test_robot_movement_discards_old_start(self):
        self.submit()
        self.manager.current_q = START + .02
        self.finish_search()
        self.assertEqual(self.observer.tick(self.manager)['disposition'], 'START_MOVED')

    def test_new_primary_planning_attempt_supersedes_old_shadow(self):
        self.submit()
        self.manager.last_graph_failure = None
        self.finish_search()
        self.assertEqual(self.observer.tick(self.manager)['disposition'],
                         'PLANNING_ATTEMPT_SUPERSEDED')

    def test_stale_feedback_prevents_admission(self):
        self.manager.feedback_ready = lambda: False
        self.assertEqual(self.observer.tick(self.manager)['reason'], 'FEEDBACK_STALE')
        self.assertIsNone(self.observer.tick(self.manager))
        self.assertFalse(self.executor.jobs)

    def test_unknown_human_is_not_ignored_by_shadow(self):
        self.p.collision_filter.enable_human_obstacles(False)
        self.assertEqual(self.observer.tick(self.manager)['reason'], 'HUMAN_VISION_UNKNOWN')
        self.assertFalse(self.executor.jobs)

    def test_valid_report_contains_no_path_and_changes_no_motion_state(self):
        self.submit_validation()
        self.finish_validation()
        result = self.observer.tick(self.manager)
        self.assertEqual(result['disposition'], 'VALID_FOR_CURRENT_SCENE_NO_DISPATCH')
        self.assertTrue(result['observation_only'])
        self.assertNotIn('path', result)
        self.assertEqual(self.manager.motion_status, 'STOPPED')
        self.assertIsNone(self.manager.pending_action)

    def test_validation_failure_cannot_be_reported_as_success(self):
        self.submit_validation()
        self.executor.jobs[-1][2].set_result(dict(
            kind='validate', reason='VALIDATION_FAILED', path=None, elapsed_s=.1))
        self.assertEqual(self.observer.tick(self.manager)['disposition'],
                         'LATEST_VALIDATION_FAILED')

    def test_small_movement_after_validation_invalidates_its_start_certificate(self):
        self.submit_validation()
        self.manager.current_q = START + .0001
        self.finish_validation()
        self.assertEqual(self.observer.tick(self.manager)['disposition'],
                         'START_CHANGED_AFTER_VALIDATION')

    def test_expired_validation_snapshot_cannot_borrow_refreshed_human_data(self):
        self.submit_validation()
        self.observer.job['scene']['human_expires_at'] = time.monotonic() - .1
        self.p.collision_filter.set_human_capsules(FAR)
        self.finish_validation()
        self.assertEqual(self.observer.tick(self.manager)['disposition'],
                         'VALIDATION_SNAPSHOT_EXPIRED')

    def test_worker_error_disables_only_observer(self):
        self.submit()
        self.executor.jobs[-1][2].set_exception(RuntimeError('worker died'))
        self.assertEqual(self.observer.tick(self.manager)['event'], 'DISABLED')
        self.assertIsNone(self.observer.tick(self.manager))
        self.assertEqual(self.manager.motion_status, 'STOPPED')

    def test_one_attempt_is_not_repeated_every_timer_tick(self):
        self.submit()
        self.executor.jobs[-1][2].set_result(dict(
            kind='search', reason='TIME_BUDGET_EXHAUSTED', path=None))
        self.observer.tick(self.manager)
        with patch('time.monotonic', return_value=time.monotonic() + 4.):
            self.assertIsNone(self.observer.tick(self.manager))
        self.assertEqual(len(self.executor.jobs), 1)

    def test_close_cancels_queued_job(self):
        self.submit()
        self.observer.close()
        self.assertTrue(self.executor.closed)
        self.assertTrue(self.executor.jobs[-1][2].cancelled())


class NativeShadowTests(unittest.TestCase):
    def test_capture_copies_obstacle_shapes_masks_poses_and_keeps_expiry(self):
        p = planner(FAR)
        checker = p.collision_filter
        scene = capture_scene(checker)
        old = scene['mocap_pos'].copy()
        checker.data.mocap_pos[:] += 10.
        checker.model.geom_size[:] *= 2.
        checker.model.geom_contype[:] = 0
        np.testing.assert_array_equal(scene['mocap_pos'], old)
        self.assertFalse(np.array_equal(scene['geom_size'], checker.model.geom_size))
        self.assertFalse(np.array_equal(scene['geom_contype'], checker.model.geom_contype))
        self.assertEqual(scene['human_received_at'], checker.human_received_at)

    def test_expired_real_job_is_rejected_without_refresh(self):
        p = planner(FAR)
        scene = capture_scene(p.collision_filter)
        scene['human_expires_at'] = time.monotonic() - .1
        result = run_job(dict(kind='search', scene=scene, sample_step=.01), p.collision_filter)
        self.assertEqual(result['reason'], 'HUMAN_VISION_UNKNOWN')
        self.assertIsNone(result['path'])

    def test_latest_geometry_validation_rejects_previously_valid_direct_path(self):
        scene = make_scenes()[1]
        p = planner(scene[1])
        result = run_job(dict(kind='validate', scene=capture_scene(p.collision_filter),
                              sample_step=.01, start=START, path=np.array([START, GOAL]),
                              max_segment_delta=p.max_segment_delta, total_budget=.5),
                         p.collision_filter)
        self.assertEqual(result['reason'], 'VALIDATION_FAILED')
        self.assertIsNotNone(result['failure'])

    def test_spawned_process_returns_report_while_parent_updates_feedback(self):
        p = planner(FAR)
        checker = p.collision_filter
        checker.human_data_timeout = 2.
        manager = manager_for(p)
        observer = ShadowRRTObserver(MODEL, p.joint_names, options(checker))
        self.addCleanup(observer.close)
        events = [observer.tick(manager)]
        count = 0
        until = time.monotonic() + 8.
        while time.monotonic() < until:
            manager.feedback_time = time.monotonic()
            checker.set_human_capsules(FAR)
            event = observer.tick(manager)
            count += 1
            if event is not None:
                events.append(event)
                if event.get('disposition') == 'VALID_FOR_CURRENT_SCENE_NO_DISPATCH':
                    break
                self.assertNotIn(event.get('event'), ('DISCARDED', 'DISABLED'), events)
            time.sleep(.005)
        self.assertEqual(events[-1].get('disposition'),
                         'VALID_FOR_CURRENT_SCENE_NO_DISPATCH', events)
        self.assertGreater(count, 1)
        self.assertNotEqual(events[-1]['worker_pid'], os.getpid())
        self.assertEqual(manager.motion_status, 'STOPPED')
        self.assertIsNone(manager.pending_action)


class ManagerObservationHookTests(unittest.TestCase):
    def setUp(self):
        from test_dynamic_replanning import DynamicReplanningTests

        helper = DynamicReplanningTests()
        helper.setUp()
        self.addCleanup(helper.doCleanups)
        self.manager = helper.make_manager()

    def test_graph_failure_records_an_attempt_without_changing_decision(self):
        manager = self.manager
        with patch.object(manager.planner, 'plan', return_value=False), \
                patch.object(manager.planner, 'find_alternative_goal_path', return_value=None), \
                patch.object(manager, '_prepare_safe_position', return_value=False):
            manager.planner.last_plan_failure = 'NO_GRAPH_PATH'
            decision = manager.step()
        self.assertEqual((decision.action, decision.status), ('NONE', 'NO_PATH'))
        self.assertEqual(manager.last_graph_failure['sequence'], 1)
        np.testing.assert_array_equal(manager.last_graph_failure['start'], manager.current_q)
        self.assertIsNone(manager.current_path)

    def test_new_target_invalidates_observation_generation(self):
        manager = self.manager
        manager.last_graph_failure = dict(sequence=1)
        old = manager.target_generation
        manager.update_target(manager.target_base + [.01, 0., 0.], [.5, .4, .3])
        self.assertEqual(manager.target_generation, old + 1)
        self.assertIsNone(manager.last_graph_failure)

    def test_successful_primary_path_does_not_request_shadow_search(self):
        self.assertEqual(self.manager.step().status, 'PATH_READY')
        self.assertIsNone(self.manager.last_graph_failure)


if __name__ == '__main__':
    unittest.main()
