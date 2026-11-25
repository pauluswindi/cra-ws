"""Primary planner routing, explicit RRT requests and shared safety gates."""

import unittest
from unittest.mock import patch

import numpy as np

from planning.astar_execution import astar_settings
from planning.planning_manager import PlanningManager
from planning.planner_modes import configured_planner, planner_selection
from planning.rrt_fallback import rrt_settings
from test_rrt_fallback import FallbackFixture, GOAL


class ConfigurationTests(unittest.TestCase):
    def test_explicit_modes_select_worker_policy(self):
        for strategy, worker in (('gng', 'off'), ('rrt', 'fallback'), ('combined', 'fallback')):
            with self.subTest(strategy=strategy):
                planning = {'planner_mode': strategy, 'rrt': {'mode': 'shadow'}}
                self.assertEqual(planner_selection(planning), (strategy, worker))
                self.assertEqual(rrt_settings(planning, 'simulation', worker)[0], worker)

    def test_invalid_mode_is_rejected(self):
        for mode in (None, True, 'combine', 'unknown', {}):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                configured_planner({'planner_mode': mode})

    def test_legacy_yaml_and_explicit_diagnostic_overrides_remain_available(self):
        self.assertEqual(planner_selection({}), (None, ''))
        self.assertEqual(planner_selection({'planner_mode': 'combined'}, 'shadow'),
                         ('gng', 'shadow'))
        self.assertEqual(planner_selection({'planner_mode': 'rrt'}, 'off'), ('gng', 'off'))
        self.assertEqual(planner_selection({'planner_mode': 'rrt'}, 'fallback'),
                         ('rrt', 'fallback'))

    def test_new_rrt_modes_remain_simulation_only(self):
        for mode in ('rrt', 'combined'):
            _, worker = planner_selection({'planner_mode': mode})
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                rrt_settings({}, 'robot', worker)
        _, worker = planner_selection({'planner_mode': 'gng'})
        self.assertEqual(rrt_settings({}, 'robot', worker)[0], 'off')


class RoutingTests(FallbackFixture):
    def test_gng_failure_never_requests_rrt_even_if_worker_flag_is_set(self):
        self.m.planner_mode = 'gng'
        self.assertEqual(self.m.step().status, 'NO_PATH')
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertEqual(self.m.rrt_block_reason, 'PLANNER_MODE_GNG')

    def test_combined_uses_valid_graph_route_without_rrt(self):
        self.m.planner_mode = 'combined'
        self.p.primary_success = True
        self.assertEqual(self.m.step().status, 'PATH_READY')
        self.assertEqual(self.m.path_source, 'GNG_ASTAR')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_combined_graph_failure_records_fallback_trigger(self):
        self.request()
        self.assertEqual(self.m.rrt_trigger, 'NO_GRAPH_PATH')

    def test_rrt_skips_a_star_even_when_graph_would_succeed(self):
        self.m.planner_mode = 'rrt'
        self.p.primary_success = True
        with patch.object(self.p, 'plan', side_effect=AssertionError('A* called')), \
                patch.object(self.p, 'find_alternative_goal_path',
                             side_effect=AssertionError('Graph alternatives called')):
            self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.assertIsNone(self.m.last_graph_failure)
        self.assertEqual(self.m.rrt_trigger, 'PRIMARY_RRT')
        self.assertEqual(self.m.rrt_fallback_attempt['trigger'], 'PRIMARY_RRT')
        self.assertTrue(self.m.rrt_fallback_pending)
        self.assertIsNone(self.m.current_path)

    def test_rrt_result_uses_the_same_execution_certificate_and_handshake(self):
        self.m.planner_mode = 'rrt'
        self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.assertTrue(self.m.offer_rrt_result(self.result()))
        self.assertEqual(self.m.step().status, 'RRT_PATH_READY')
        self.assertEqual(self.m.path_source, 'RRT_CONNECT')
        self.assertEqual(self.m.step().action, 'EXECUTE')

    def test_rrt_no_goal_does_not_request_search(self):
        self.m.planner_mode = 'rrt'
        with patch.object(self.p, 'select_goal', return_value=None), \
                patch.object(self.p, 'plan', side_effect=AssertionError('A* called')):
            self.assertEqual(self.m.step().status, 'GOAL_UNREACHABLE')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_rrt_unknown_does_not_borrow_simulation_override(self):
        self.m.planner_mode = 'rrt'
        self.m.human_status = 'UNKNOWN'
        self.p.collision_filter.human_status = 'UNKNOWN'
        self.m.step()
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertIsNone(self.m.last_graph_failure)

    def test_rrt_rejects_invalid_goal_without_search_or_execution(self):
        self.m.planner_mode = 'rrt'
        with patch.object(self.p, 'select_goal', return_value=np.full(5, 3.)):
            decision = self.m.step()
        self.assertEqual(decision.status, 'GOAL_UNREACHABLE')
        self.assertEqual(decision.action, 'NONE')
        self.assertFalse(self.m.rrt_fallback_pending)

    def test_rrt_manual_stop_cannot_be_released(self):
        self.m.planner_mode = 'rrt'
        self.m.motion_status = 'STOPPED'
        self.m.stop_owned = False
        decision = self.m.step()
        self.assertNotEqual(decision.action, 'RELEASE')
        self.assertFalse(self.m.rrt_fallback_pending)
        self.assertEqual(self.m.rrt_block_reason, 'EXTERNAL_STOP')

    def test_rrt_input_refresh_preserves_primary_request_without_graph_retries(self):
        self.m.planner_mode = 'rrt'

        def slow_goal(*_):
            self.now[0] += 1.21
            return GOAL.copy()

        with patch.object(self.p, 'select_goal', side_effect=slow_goal) as select, \
                patch.object(self.p, 'plan', side_effect=AssertionError('A* called')):
            self.assertEqual(self.m.step().status, 'RRT_WAITING_FOR_FEEDBACK')
            self.m.feedback_time = self.m.obstacle_time = self.now[0]
            self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.assertEqual(select.call_count, 1)
        self.assertIsNone(self.m.last_graph_failure)

    def test_rrt_search_failure_does_not_start_graph_safe_position_search(self):
        self.m.planner_mode = 'rrt'
        self.assertEqual(self.m.step().status, 'RRT_SEARCHING')
        self.m.offer_rrt_result(self.result(reason='NO_PATH'))
        self.assertEqual(self.m.step().status, 'RRT_FAILED')
        self.safe.assert_not_called()


class IncrementalRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_goal_selection as fixtures
        cls.fixture_type = fixtures.NativeGoalTests
        if not hasattr(cls.fixture_type, 'base'):
            cls.fixture_type.setUpClass()

    def setUp(self):
        fixture = self.fixture_type('test_graph_goal_priority_never_invokes_ik')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture = fixture

    def manager(self, gap=False):
        planner, target, world = self.fixture.planner(gap=gap)
        planner.planner_mode = 'rrt'
        planner.astar_execution_config = astar_settings({'astar': {
            'mode': 'incremental', 'max_operations_per_slice': 1}})
        manager = PlanningManager(planner, execute_path=True, human_avoidance_enabled=True,
                                  simulation_only_ignore_unknown=True)
        from test_dynamic_replanning import START, FAR
        manager.update_joint_state(START)
        manager.update_obstacles([])
        manager.update_target(target, world)
        manager.update_motion_status('IDLE')
        manager.update_human_snapshot(FAR)
        manager.rrt_fallback_enabled = True
        return manager

    def test_goal_only_slices_never_run_graph_search(self):
        manager = self.manager()
        private = manager.graph_workspace.planner
        with patch.object(private, 'plan_steps', side_effect=AssertionError('A* called')), \
                patch.object(private.planner, 'plan_steps',
                             side_effect=AssertionError('Graph search called')):
            self.assertEqual(manager.step().status, 'GOAL_SEARCHING')
            manager.advance_graph_search()
            self.assertFalse(manager.rrt_fallback_pending)
            for _ in range(10000):
                manager.advance_graph_search()
                if manager.graph_search.done:
                    break
            self.assertEqual(manager.step().status, 'RRT_SEARCHING')
        self.assertEqual(manager.last_graph_search['task'], 'GOAL_SELECTION')
        self.assertIsNone(manager.last_graph_failure)

    def test_cancelled_goal_search_cannot_launch_rrt_for_old_target(self):
        manager = self.manager()
        manager.step()
        job = manager.graph_search
        manager.update_target(manager.target_base + [.02, 0., 0.], [1., 1., 1.])
        self.assertIsNone(manager.graph_search)
        self.assertIsNone(job.steps.gi_frame)
        self.assertFalse(manager.rrt_fallback_pending)
        self.assertIsNone(manager.rrt_primary_attempt)

    def test_no_goal_after_incremental_selection_does_not_request_rrt(self):
        manager = self.manager(gap=True)
        manager.graph_workspace.live_planner.goal_selector.settings['ik_enabled'] = False
        self.assertEqual(manager.step().status, 'GOAL_SEARCHING')
        for _ in range(10000):
            manager.advance_graph_search()
            if manager.graph_search.done:
                break
        self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
        self.assertFalse(manager.rrt_fallback_pending)


if __name__ == '__main__':
    unittest.main()
