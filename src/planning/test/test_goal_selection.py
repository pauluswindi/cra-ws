"""Goal diagnostics, bounded IK and graph-first recovery with native CRA geometry.

Sparse roadmap fixtures and injected rejection reports are controlled cases,
not recordings of Kinect or proof of moving-human performance.
"""

from collections import OrderedDict
from copy import copy
import contextlib
import io
import unittest
from unittest.mock import patch

import mujoco
import numpy as np
import yaml

from planning.algorithms.a_star import AStarPlanner
from planning.astar_execution import astar_settings
from planning.goal_selection import (
    GoalSelector, goal_settings, position, position_and_jacobian, solve_position_steps,
)
from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from planning.path_planner import PathPlanner
from planning.planning_manager import PlanningManager
from planning.search_steps import complete
from simulation.frame_transform import FrameTransform
from test_dynamic_replanning import ROOT, FAR, UNKNOWN, START


class SettingsTests(unittest.TestCase):
    def test_ik_requires_explicit_enablement_and_simulation(self):
        config = {'robot': {'mode': 'simulation'}}
        self.assertFalse(goal_settings(config)['ik_enabled'])
        config['planning'] = {'goal_selection': {'ik_fallback': {'enabled': True}}}
        self.assertTrue(goal_settings(config)['ik_enabled'])
        config['robot']['mode'] = 'robot'
        self.assertFalse(goal_settings(config)['ik_enabled'])

    def test_invalid_settings_are_rejected(self):
        for section in ({'retain_valid_goal': 1}, {'clearance_candidates': 0},
                        {'preferred_clearance': float('nan')},
                        {'ik_fallback': {'enabled': 'true'}},
                        {'ik_fallback': {'max_iterations': 1.5}},
                        {'ik_fallback': {'time_budget_ms': 0.}},
                        {'ik_fallback': {'retry_interval': -1.}}):
            with self.subTest(section=section), self.assertRaises(ValueError):
                goal_settings({'planning': {'goal_selection': section}})


class NativeGoalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load((ROOT / 'src/cra/config/robot.yaml').read_text())
        cls.base = PathPlanner(ROOT / 'src/planning/data/workspace_graph.npz',
                               ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml',
                               ROOT / 'src/cra/config/robot.yaml')

    def setUp(self):
        clock = patch('time.monotonic', return_value=100.)
        self.now = clock.start()
        self.addCleanup(clock.stop)
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def planner(self, gap=False):
        planner = copy(self.base)
        # Graph-first goal tests must not inherit the operator's RRT-only mode.
        planner.planner_mode = 'combined'
        checker = copy(self.base.collision_filter)
        checker.model = copy(checker.model)
        checker.data = mujoco.MjData(checker.model)
        checker._human_report_cache = OrderedDict()
        checker.human_data_timeout, checker.human_safety_margin = 2., .05
        checker.enable_human_obstacles(True)
        checker.set_human_capsules(FAR)
        checker._set_joint_configuration(START)
        planner.collision_filter = checker
        planner.goal_selector = GoalSelector(self.config)
        planner.goal_seed = START.copy()
        planner.planner = copy(self.base.planner)
        planner.planner.checker = checker
        planner.planner.state_cache, planner.planner.edge_cache = {}, {}
        planner.planner.revision = None
        if gap:
            planner.joint_q = np.array([START])
            planner.joint_eof = np.array([position(checker, checker.model.body(
                planner.goal_selector.body_name).id)])
            planner.joint_edges = np.empty((0, 2), dtype=int)
            planner.joint_costs = np.empty(0)
            planner.planner = AStarPlanner(planner.joint_q, planner.joint_edges,
                                           planner.joint_costs, checker, max_joint_delta=.2)
            planner.max_joint_delta = .2
        world = checker.data.mocap_pos[checker.obstacle_geoms['target']['mocap_id']].copy()
        target = FrameTransform.mujoco_base_to_robot_base(
            checker.data.xmat[checker.base_id].reshape(3, 3).T
            @ (world - checker.data.xpos[checker.base_id]))
        return planner, target, world

    def test_position_jacobian_matches_finite_differences_in_cra_base(self):
        planner, _, _ = self.planner()
        checker = planner.collision_filter
        body = checker.model.body(planner.goal_selector.body_name).id
        _, jacobian = position_and_jacobian(checker, body)
        numerical = np.empty((3, 5))
        for index in range(5):
            delta = np.eye(5)[index] * 1e-6
            checker._set_joint_configuration(START + delta)
            plus = position(checker, body)
            checker._set_joint_configuration(START - delta)
            minus = position(checker, body)
            numerical[:, index] = (plus - minus) / 2e-6
        np.testing.assert_allclose(jacobian, numerical, atol=1e-7, rtol=1e-5)

    def test_position_ik_solves_a_known_fk_point_and_restores_data(self):
        planner, _, _ = self.planner()
        checker = planner.collision_filter
        body = checker.model.body(planner.goal_selector.body_name).id
        wanted = START + [.15, .05, -.04, .03, -.06]
        checker._set_joint_configuration(wanted)
        point = position(checker, body)
        checker._set_joint_configuration(START)
        solution = complete(solve_position_steps(
            checker, body, point, START, planner.goal_selector.settings, lambda: True))
        self.assertIsNotNone(solution)
        np.testing.assert_array_equal(checker.data.qpos[checker.qpos_addresses], START)
        self.assertTrue(np.all(solution >= checker.lower) and np.all(solution <= checker.upper))
        checker._set_joint_configuration(solution)
        self.assertLessEqual(np.linalg.norm(position(checker, body) - point), .001)

    def test_graph_goal_priority_never_invokes_ik(self):
        planner, target, _ = self.planner()
        with patch('planning.goal_selection.solve_position_steps') as ik:
            q = planner.select_goal(target, .12, .04)
        self.assertIsNotNone(q)
        self.assertEqual(planner.goal_selector.report['source'], 'GRAPH')
        ik.assert_not_called()

    def test_valid_retained_goal_is_rechecked_and_reused(self):
        planner, target, _ = self.planner()
        first = planner.select_goal(target, .12, .04)
        planner.goal_selector.remember(first)
        with patch.object(planner.collision_filter, 'is_state_valid',
                          wraps=planner.collision_filter.is_state_valid) as check:
            second = planner.select_goal(target, .12, .04)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(planner.goal_selector.report['reason'], 'RETAINED_GOAL')
        self.assertEqual(check.call_count, 1)

    def test_invalid_retained_goal_is_replaced(self):
        planner, target, _ = self.planner()
        first = planner.select_goal(target, .12, .04)
        planner.goal_selector.remember(first)
        checker = planner.collision_filter
        original = checker.is_state_valid

        def check(q):
            if np.array_equal(q, first):
                return checker._state_validation_failed(q, 'HUMAN_COLLISION')
            return original(q)

        with patch.object(checker, 'is_state_valid', side_effect=check):
            second = planner.select_goal(target, .12, .04)
        self.assertIsNotNone(second)
        self.assertFalse(np.array_equal(first, second))

    def test_target_change_clears_old_goal_and_ik_cache(self):
        planner, target, _ = self.planner()
        first = planner.select_goal(target, .12, .04)
        planner.goal_selector.remember(first)
        planner.goal_selector.cached_ik = [first.copy()]
        planner.goal_selector.settings['ik_enabled'] = False
        planner.select_goal(target + [2., 0., 0.], .12, .04)
        self.assertIsNone(planner.goal_selector.last_goal)
        self.assertFalse(planner.goal_selector.cached_ik)

    def test_rejection_diagnostics_reuse_existing_check_results(self):
        planner, target, _ = self.planner()
        planner.goal_selector.settings['ik_enabled'] = False
        checker = planner.collision_filter

        def reject(q):
            return checker._state_validation_failed(q, 'HUMAN_COLLISION',
                                                    human={'clearance': .03})

        with patch.object(checker, 'is_state_valid', side_effect=reject) as check, \
                patch.object(checker, 'diagnose_state', side_effect=AssertionError('Repeated FK')):
            self.assertIsNone(planner.select_goal(target, .12, .04))
        report = planner.goal_selector.report
        self.assertEqual(report['reason'], 'GRAPH_GOALS_BLOCKED_HUMAN')
        self.assertEqual(report['window_candidates'], check.call_count)
        self.assertEqual(report['rejected']['HUMAN_COLLISION'], check.call_count)
        self.assertLessEqual(len(report['examples']), 3)

    def test_clearance_ranking_uses_existing_report_and_keeps_hard_checks(self):
        planner, target, _ = self.planner()
        planner.goal_selector.settings['clearance_candidates'] = 1
        candidates = list(planner.goal_candidates(target, .12, .04))[:2]
        self.assertEqual(len(candidates), 2)
        checker = planner.collision_filter
        original = checker.is_state_valid

        def check(q):
            valid = original(q)
            if valid:
                checker.last_state_validation_clearance = (
                    .09 if np.array_equal(q, candidates[1]) else .055)
            return valid

        planner.goal_selector.settings['clearance_candidates'] = 4
        with patch.object(checker, 'is_state_valid', side_effect=check), \
                patch.object(checker, 'human_collision_report',
                             side_effect=AssertionError('Repeated geometry query')):
            selected = planner.select_goal(target, .12, .04)
        np.testing.assert_array_equal(selected, candidates[1])
        self.assertEqual(checker.human_safety_margin, .05)

    def test_unknown_input_cannot_trigger_ik_under_simulation_override(self):
        planner, target, _ = self.planner(gap=True)
        planner.collision_filter.set_human_capsules(UNKNOWN)
        planner.collision_filter.enable_human_obstacles(False)
        with patch('planning.goal_selection.solve_position_steps') as ik:
            self.assertIsNone(planner.select_goal(target, .12, .04))
        ik.assert_not_called()
        self.assertEqual(planner.goal_selector.report['reason'], 'IK_INPUT_NOT_READY')

    def test_only_fresh_certified_empty_can_supply_ik_without_observed_humans(self):
        planner, target, _ = self.planner(gap=True)
        planner.collision_filter.set_human_capsules(
            HumanSnapshot('EMPTY', 'NO_ACTIVE_CAPSULES', ()))
        planner.collision_filter.enable_human_obstacles(False)
        planner.goal_empty_until = 101.
        self.assertIsNotNone(planner.select_goal(target, .12, .04), planner.goal_selector.report)
        self.now.return_value = 101.1
        self.assertIsNone(planner.select_goal(target, .12, .04))
        self.assertEqual(planner.goal_selector.report['reason'], 'IK_INPUT_NOT_READY')

    def test_ik_can_replace_rejected_graph_goals_with_a_new_native_valid_configuration(self):
        planner, target, _ = self.planner()
        checker = planner.collision_filter
        original = checker.is_state_valid

        def reject_graph_only(q):
            if np.any(np.max(np.abs(planner.joint_q - q), axis=1) < 1e-8):
                return checker._state_validation_failed(q, 'HUMAN_COLLISION')
            return original(q)

        with patch.object(checker, 'is_state_valid', side_effect=reject_graph_only):
            selected = planner.select_goal(target, .12, .04)
        self.assertIsNotNone(selected, planner.goal_selector.report)
        self.assertEqual(planner.goal_selector.report['graph_outcome'], 'GRAPH_GOALS_BLOCKED_HUMAN')
        self.assertEqual(planner.goal_selector.report['source'], 'IK')
        self.assertTrue(original(selected))

    def test_ik_goal_still_uses_graph_when_a_connection_is_valid(self):
        for mode in ('blocking', 'incremental'):
            with self.subTest(mode=mode):
                planner, _, _ = self.planner(gap=True)
                checker = planner.collision_filter
                body = checker.model.body(planner.goal_selector.body_name).id
                target = position(checker, body) + [.15, 0., 0.]
                world = (checker.data.xpos[checker.base_id]
                         + checker.data.xmat[checker.base_id].reshape(3, 3)
                         @ FrameTransform.robot_base_to_mujoco_base(target))
                planner.max_joint_delta = planner.planner.max_joint_delta = .8
                planner.astar_execution_config = astar_settings({'astar': {'mode': mode}})
                manager = PlanningManager(planner, execute_path=True, human_avoidance_enabled=True,
                                          simulation_only_ignore_unknown=True)
                manager.update_joint_state(START)
                manager.update_obstacles([])
                manager.update_target(target, world)
                manager.update_motion_status('IDLE')
                manager.update_human_snapshot(FAR)
                manager.rrt_fallback_enabled = True
                result = manager.step()
                if mode == 'incremental':
                    for _ in range(10000):
                        manager.advance_graph_search()
                        if manager.graph_search.done:
                            break
                    result = manager.step()
                self.assertEqual(result.status, 'PATH_READY', planner.goal_selector.report)
                self.assertEqual(planner.goal_selector.report['source'], 'IK')
                self.assertEqual(manager.path_source, 'GNG_ASTAR')
                self.assertFalse(manager.rrt_fallback_pending)

    def test_ik_budget_exhaustion_is_not_a_path_failure(self):
        planner, target, _ = self.planner(gap=True)
        with patch('planning.goal_selection.time.perf_counter', side_effect=[0., 1., 1., 1.]):
            self.assertIsNone(planner.select_goal(target, .12, .04))
        self.assertEqual(planner.goal_selector.report['reason'], 'IK_BUDGET_EXHAUSTED')
        self.assertNotEqual(planner.last_plan_failure, 'NO_GRAPH_PATH')

    def test_failed_ik_is_cached_and_scene_changes_obey_cooldown(self):
        planner, target, _ = self.planner(gap=True)

        def no_solution(*_):
            yield
            return None

        with patch('planning.goal_selection.solve_position_steps', side_effect=no_solution) as ik:
            self.assertIsNone(planner.select_goal(target, .12, .04))
            calls = ik.call_count
            self.assertGreater(calls, 0)
            self.assertIsNone(planner.select_goal(target, .12, .04))
            self.assertEqual(ik.call_count, calls)
            self.assertEqual(planner.goal_selector.report['reason'], 'IK_SCENE_ALREADY_TRIED')
            planner.collision_filter.set_human_capsules(HumanSnapshot(
                'OBSERVED', 'CAPSULES_PRESENT', (
                    HumanCapsule(1, 0, (12., 12., 12.), (12., 12., 13.), .1),)))
            self.assertIsNone(planner.select_goal(target, .12, .04))
            self.assertEqual(planner.goal_selector.report['reason'], 'IK_RETRY_COOLDOWN')
            self.assertEqual(ik.call_count, calls)
            self.now.return_value = 101.1
            self.assertTrue(planner.goal_selector.retry_due(101.1))
            self.assertIsNone(planner.select_goal(target, .12, .04))
            self.assertGreater(ik.call_count, calls)

    def test_ik_aborts_when_input_expires_between_iterations(self):
        planner, _, _ = self.planner()
        checker = planner.collision_filter
        body = checker.model.body(planner.goal_selector.body_name).id
        steps = solve_position_steps(checker, body, np.array([10., 10., 10.]), START,
                                     planner.goal_selector.settings, planner.human_data_ready)
        next(steps)
        self.now.return_value = 102.1
        self.assertIsNone(complete(steps))
        np.testing.assert_array_equal(checker.data.qpos[checker.qpos_addresses], START)

    def test_ik_supplies_a_valid_goal_when_sparse_graph_has_no_sample(self):
        planner, target, _ = self.planner(gap=True)
        q = planner.select_goal(target, .12, .04)
        report = planner.goal_selector.report
        self.assertIsNotNone(q, report)
        self.assertEqual(report['graph_outcome'], 'NO_GRAPH_SAMPLES')
        self.assertEqual(report['source'], 'IK')
        checker = planner.collision_filter
        self.assertTrue(checker.is_state_valid(q))
        distance = np.linalg.norm(position(checker, checker.model.body(
            planner.goal_selector.body_name).id) - target)
        self.assertTrue(.04 <= distance <= .12)
        self.assertLessEqual(report['ik_attempts'], 6)

    def test_cached_ik_is_rechecked_without_solving_again(self):
        planner, target, _ = self.planner(gap=True)
        first = planner.select_goal(target, .12, .04)
        self.assertIsNotNone(first, planner.goal_selector.report)
        with patch('planning.goal_selection.solve_position_steps') as ik, \
                patch.object(planner.collision_filter, 'is_state_valid',
                             wraps=planner.collision_filter.is_state_valid) as check:
            second = planner.select_goal(target, .12, .04)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(planner.goal_selector.report['reason'], 'CACHED_IK_GOAL')
        ik.assert_not_called()
        self.assertTrue(check.called)

    def test_invalid_cached_ik_goal_is_never_returned(self):
        planner, target, _ = self.planner(gap=True)
        first = planner.select_goal(target, .12, .04)
        self.assertIsNotNone(first)
        checker = planner.collision_filter
        with patch.object(checker, 'is_state_valid', side_effect=lambda q:
                          checker._state_validation_failed(q, 'HUMAN_COLLISION')):
            self.assertIsNone(planner.select_goal(target, .12, .04))
        self.assertEqual(planner.goal_selector.report['ik_cache_hits'], 0)
        self.assertGreater(planner.goal_selector.report['rejected']['HUMAN_COLLISION'], 0)

    def test_manager_retries_once_after_ik_cooldown_even_without_another_scene_change(self):
        planner, target, world = self.planner(gap=True)
        planner.astar_execution_config = astar_settings({'astar': {'mode': 'blocking'}})
        manager = PlanningManager(planner, execute_path=True, human_avoidance_enabled=True,
                                  simulation_only_ignore_unknown=True)
        manager.update_joint_state(START)
        manager.update_obstacles([])
        manager.update_target(target, world)
        manager.update_motion_status('IDLE')
        manager.update_human_snapshot(FAR)

        def no_solution(*_):
            yield
            return None

        with patch('planning.goal_selection.solve_position_steps', side_effect=no_solution) as ik:
            self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
            calls = ik.call_count
            manager.update_human_snapshot(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(1, 0, (12., 12., 12.), (12., 12., 13.), .1),)))
            self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
            self.assertEqual(planner.goal_selector.report['reason'], 'IK_RETRY_COOLDOWN')
            self.assertEqual(ik.call_count, calls)
            self.now.return_value = 101.1
            manager.update_joint_state(START)
            manager.update_obstacles([])
            self.assertEqual(manager.step().status, 'GOAL_UNREACHABLE')
            self.assertGreater(ik.call_count, calls)

    def test_graph_first_then_rrt_with_an_ik_goal_in_both_modes(self):
        for mode in ('blocking', 'incremental'):
            with self.subTest(mode=mode):
                planner, target, world = self.planner(gap=True)
                planner.astar_execution_config = astar_settings({'astar': {'mode': mode}})
                manager = PlanningManager(planner, execute_path=True, human_avoidance_enabled=True,
                                          simulation_only_ignore_unknown=True)
                manager.update_joint_state(START)
                manager.update_obstacles([])
                manager.update_target(target, world)
                manager.update_motion_status('IDLE')
                manager.update_human_snapshot(FAR)
                manager.rrt_fallback_enabled = True
                result = manager.step()
                if mode == 'incremental':
                    self.assertEqual(result.status, 'ASTAR_SEARCHING')
                    for _ in range(10000):
                        manager.advance_graph_search()
                        if manager.graph_search.done:
                            break
                    result = manager.step()
                self.assertEqual(result.status, 'RRT_SEARCHING', planner.goal_selector.report)
                self.assertEqual(planner.goal_selector.report['source'], 'IK')
                self.assertEqual(planner.last_plan_failure, 'NO_GRAPH_PATH')
                self.assertTrue(manager.rrt_fallback_pending)
                self.assertEqual(result.action, 'NONE')


if __name__ == '__main__':
    unittest.main()
