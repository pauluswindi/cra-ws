"""Cooperative search equivalence and graph-first safety with native geometry.

The gap fixture uses synthetic roadmap/Cartesian samples. Motion acknowledgements
are supplied explicitly; these tests do not establish live Kinect performance.
"""

from collections import OrderedDict
from copy import copy
import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from planning.algorithms.a_star import AStarPlanner
from planning.astar_execution import GraphSearch, astar_settings
from planning.path_planner import PathPlanner
from planning.planning_manager import PlanningManager
from planning.search_steps import complete
from simulation.frame_transform import FrameTransform
from test_astar_responsiveness import Checker, q
from test_dynamic_replanning import FAR, UNKNOWN, ROOT, START


class SettingsTests(unittest.TestCase):
    def test_missing_section_preserves_blocking(self):
        self.assertEqual(astar_settings({})['mode'], 'blocking')
        self.assertEqual(astar_settings({})['max_operations_per_slice'], 2048)

    def test_yaml_selects_incremental(self):
        self.assertEqual(astar_settings({'astar': {'mode': 'incremental'}})['mode'],
                         'incremental')

    def test_invalid_values_fail_at_startup(self):
        for values in ({'mode': 'rrt'}, {'mode': True}, {'slice_budget_ms': 0},
                       {'slice_period_ms': float('nan')}, {'max_operations_per_slice': 1.5},
                       {'slice_budget_ms': True}, {'slice_period_ms': '10'}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                astar_settings({'astar': values})
        with self.assertRaises(ValueError):
            astar_settings({'astar': []})


class SliceAccountingTests(unittest.TestCase):
    def job(self, limit=4):
        workspace = SimpleNamespace(capture=lambda _: None,
                                    planner=SimpleNamespace(last_plan_failure='NONE'))
        job = GraphSearch(workspace, None, np.zeros(5), np.zeros(3), 1, .12, .04,
                          astar_settings({'astar': {'max_operations_per_slice': limit}}))
        job.steps = (None for _ in range(10000))
        self.addCleanup(job.close)
        return job

    def test_operation_limit_yields_without_claiming_search_failure(self):
        job = self.job()
        with patch('time.perf_counter', return_value=0.):
            job.advance()
        stats = job.statistics()
        self.assertFalse(job.done)
        self.assertEqual(stats['last_yield_reason'], 'OPERATION_LIMIT')
        self.assertEqual(stats['last_slice_operations'], 4)
        self.assertEqual(stats['operations_total'], 4)
        self.assertEqual(stats['operation_limit_yields'], 1)
        self.assertEqual(stats['time_budget_yields'], 0)
        self.assertIsNone(stats['search_failure'])

    def test_time_budget_yields_before_operation_cap(self):
        job = self.job(limit=2048)
        with patch('time.perf_counter', side_effect=[0., .001, .002, .011, .012]):
            job.advance()
        self.assertFalse(job.done)
        self.assertEqual(job.statistics()['last_yield_reason'], 'TIME_BUDGET')
        self.assertEqual(job.operations_total, 3)
        self.assertEqual(job.time_budget_yields, 1)
        self.assertEqual(job.operation_limit_yields, 0)

    def test_completion_is_not_counted_as_a_budget_yield(self):
        job = self.job()
        job.steps = (None for _ in range(1))
        with patch('time.perf_counter', return_value=0.):
            job.advance()
        self.assertTrue(job.done)
        self.assertEqual(job.last_yield_reason, 'COMPLETE')
        self.assertEqual(job.operations_total, 1)
        self.assertEqual(job.time_budget_yields + job.operation_limit_yields, 0)


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def planner(self, checker):
        points = np.array([q(0), q(1, .2), q(1, -1), q(2)])
        edges = np.array([[0, 1], [1, 3], [0, 2], [2, 3]])
        costs = np.linalg.norm(points[edges[:, 0]] - points[edges[:, 1]], axis=1)
        return AStarPlanner(points, edges, costs, checker, max_joint_delta=.25,
                            connection_count=1)

    def test_suspended_search_matches_blocking_with_a_blocked_branch(self):
        first, second = Checker(), Checker()
        for checker in (first, second):
            checker.blocked_edges.add(tuple(sorted((tuple(q(0)), tuple(q(1, .2))))))
        expected = self.planner(first).plan(q(0), q(2))
        steps = self.planner(second).plan_steps(q(0), q(2))
        slices = 0
        while True:
            try:
                next(steps)
                slices += 1
            except StopIteration as result:
                np.testing.assert_array_equal(result.value, expected)
                break
        self.assertGreater(slices, 10)
        for a, b in zip(expected[:-1], expected[1:]):
            self.assertIn(tuple(sorted((tuple(a), tuple(b)))), second.certified_edges)

    def test_expiry_between_slices_never_returns_a_route(self):
        checker = Checker()
        checker.human_obstacles_enabled = True
        steps = self.planner(checker).plan_steps(q(0), q(2))
        for _ in range(5):
            next(steps)
        checker.ready = False
        self.assertIsNone(complete(steps))

    def test_scene_revision_between_slices_invalidates_the_search(self):
        checker = Checker()
        steps = self.planner(checker).plan_steps(q(0), q(2))
        next(steps)
        checker.environment_revision += 1
        with self.assertRaisesRegex(RuntimeError, 'Obstacle changed'):
            complete(steps)


class NativeExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = PathPlanner(
            ROOT / 'src/planning/data/workspace_graph.npz',
            ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml',
            ROOT / 'src/cra/config/robot.yaml')

    def setUp(self):
        self.clock = patch('time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def manager(self, mode='incremental', gap=False):
        planner = copy(self.base)
        planner.goal_selector = self.base.goal_selector.fork()
        checker = copy(self.base.collision_filter)
        checker.model = copy(checker.model)
        checker.data = mujoco.MjData(checker.model)
        checker._human_report_cache = OrderedDict()
        mujoco.mj_forward(checker.model, checker.data)
        planner.collision_filter = checker
        planner.planner = copy(self.base.planner)
        planner.planner.checker = checker
        planner.planner.state_cache, planner.planner.edge_cache = {}, {}
        planner.planner.revision = None
        planner.astar_execution_config = astar_settings({'astar': {
            'mode': mode, 'max_operations_per_slice': 1}})
        if gap:
            planner.joint_q = np.array([START, START + [.9, 0., 0., 0., 0.]])
            planner.joint_edges = np.empty((0, 2), dtype=np.int64)
            planner.joint_costs = np.empty(0)
            planner.planner = AStarPlanner(planner.joint_q, planner.joint_edges,
                                           planner.joint_costs, checker, max_joint_delta=.8)
        checker.human_data_timeout = 2.
        checker.human_safety_margin = .05
        target_world = checker.data.mocap_pos[checker.obstacle_geoms['target']['mocap_id']].copy()
        target = FrameTransform.mujoco_base_to_robot_base(
            checker.data.xmat[checker.base_id].reshape(3, 3).T
            @ (target_world - checker.data.xpos[checker.base_id]))
        if gap:
            planner.joint_eof = np.array([target + [1., 0., 0.], target + [.06, 0., 0.]])
        # Exercise graph scheduling independently of the operator's selected mode.
        planner.planner_mode = 'combined'
        manager = PlanningManager(planner, execute_path=True, human_avoidance_enabled=True,
                                  simulation_only_ignore_unknown=True)
        manager.update_joint_state(START if gap else np.zeros(5))
        manager.update_obstacles([])
        manager.update_target(target, target_world)
        manager.update_motion_status('IDLE')
        manager.update_human_snapshot(FAR)
        manager.rrt_fallback_enabled = True
        return manager

    def finish(self, manager):
        for _ in range(100000):
            if manager.graph_search is None or manager.graph_search.done:
                break
            manager.advance_graph_search()
        else:
            self.fail('Search did not complete')
        return manager.step()

    def test_graph_route_matches_blocking_and_executes_without_rrt(self):
        baseline = self.manager('blocking')
        self.assertEqual(baseline.step().status, 'PATH_READY')
        manager = self.manager()
        self.assertEqual(manager.step().status, 'ASTAR_SEARCHING')
        manager.advance_graph_search()
        self.assertEqual(manager.step().status, 'ASTAR_SEARCHING')
        self.assertFalse(manager.rrt_fallback_pending)
        self.assertEqual(self.finish(manager).status, 'PATH_READY')
        np.testing.assert_array_equal(manager.current_path, baseline.current_path)
        self.assertEqual(manager.path_source, 'GNG_ASTAR')
        self.assertEqual(manager.graph_attempt_sequence, 1)
        self.assertEqual(manager.step().action, 'EXECUTE')
        self.assertFalse(manager.rrt_fallback_pending)

    def test_completed_graph_failure_can_request_rrt(self):
        manager = self.manager(gap=True)
        self.assertEqual(manager.step().status, 'ASTAR_SEARCHING')
        for _ in range(10):
            manager.advance_graph_search()
            self.assertFalse(manager.rrt_fallback_pending)
        self.assertEqual(self.finish(manager).status, 'RRT_SEARCHING')
        self.assertTrue(manager.rrt_fallback_pending)
        self.assertEqual(manager.graph_attempt_sequence, 1)

    def test_target_change_closes_the_old_generator(self):
        manager = self.manager()
        manager.step()
        job = manager.graph_search
        manager.update_target(manager.target_base + [.02, 0., 0.], [1., 1., 1.])
        self.assertIsNone(manager.graph_search)
        self.assertIsNone(job.steps.gi_frame)
        self.assertFalse(manager.rrt_fallback_pending)

    def test_start_movement_cancels_without_adopting_a_path(self):
        manager = self.manager()
        manager.step()
        manager.update_joint_state(manager.current_q + [.02, 0., 0., 0., 0.])
        manager.advance_graph_search()
        self.assertIsNone(manager.graph_search)
        self.assertEqual(manager.last_graph_search['reason'], 'START_MOVED')
        self.assertIsNone(manager.current_path)

    def test_unknown_humans_cancel_even_when_simulation_override_exists(self):
        manager = self.manager()
        manager.step()
        manager.update_human_snapshot(UNKNOWN)
        manager.advance_graph_search()
        self.assertIsNone(manager.graph_search)
        self.assertEqual(manager.step().status, 'HUMAN_VISION_UNKNOWN')
        self.assertFalse(manager.rrt_fallback_pending)

    def test_manual_stop_does_not_trigger_release_or_rrt(self):
        manager = self.manager()
        manager.step()
        manager.update_motion_status('STOPPED')
        manager.advance_graph_search()
        self.assertIsNone(manager.graph_search)
        self.assertEqual(manager.last_graph_search['reason'], 'EXTERNAL_STOP')
        manager.update_joint_state(manager.current_q)
        self.assertNotEqual(manager.step().action, 'RELEASE')
        self.assertIsNone(manager.graph_search)
        self.assertFalse(manager.rrt_fallback_pending)

    def test_live_updates_do_not_mutate_snapshot_or_justify_stale_graph_failure(self):
        manager = self.manager(gap=True)
        manager.step()
        job = manager.graph_search
        snapshot = job.planner.collision_filter
        original = snapshot.human_capsules
        from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
        manager.update_human_snapshot(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 0, (12., 12., 12.), (12., 12., 13.), .1),)))
        self.assertEqual(snapshot.human_capsules, original)
        self.assertEqual(self.finish(manager).status, 'ASTAR_REPLANNING')
        self.assertEqual(manager.last_graph_search['reason'], 'SCENE_CHANGED')
        self.assertFalse(manager.rrt_fallback_pending)
        self.assertIsNone(manager.last_graph_failure)

    def test_snapshot_deadline_is_not_extended_by_new_frames(self):
        manager = self.manager()
        manager.step()
        job = manager.graph_search
        received = job.planner.collision_filter.human_received_at
        self.now.return_value = 101.9
        manager.update_joint_state(manager.current_q)
        manager.update_obstacles([])
        manager.update_human_snapshot(FAR)
        self.assertEqual(job.planner.collision_filter.human_received_at, received)
        self.now.return_value = 102.1
        self.assertEqual(self.finish(manager).status, 'ASTAR_REPLANNING')
        self.assertEqual(manager.last_graph_search['reason'], 'SNAPSHOT_EXPIRED')
        self.assertFalse(manager.rrt_fallback_pending)

    def test_snapshot_expiry_at_start_check_is_not_live_start_invalidity(self):
        manager = self.manager()
        manager.step()
        for _ in range(10000):
            manager.advance_graph_search()
            if manager.graph_search.goal is not None:
                break
        self.assertIsNotNone(manager.graph_search.goal)
        self.now.return_value = 102.1
        manager.update_joint_state(manager.current_q)
        manager.update_obstacles([])
        manager.update_human_snapshot(FAR)
        with patch.object(manager, '_prepare_safe_recovery') as recovery:
            self.assertEqual(self.finish(manager).status, 'ASTAR_REPLANNING')
            recovery.assert_not_called()
        self.assertEqual(manager.last_graph_search['reason'], 'SNAPSHOT_EXPIRED')
        self.assertEqual(manager.last_graph_search['search_failure'], 'START_INVALID')
        self.assertFalse(manager.rrt_fallback_pending)

    def test_native_model_and_graph_are_reused_between_searches(self):
        manager = self.manager()
        workspace = manager.graph_workspace
        self.assertIs(workspace.planner.joint_q, manager.planner.joint_q)
        self.assertIs(workspace.planner.planner.adjacency, manager.planner.planner.adjacency)
        self.assertIsNot(workspace.planner.collision_filter.model,
                         manager.planner.collision_filter.model)
        manager.step()
        self.finish(manager)
        model = workspace.planner.collision_filter.model
        manager.invalidate_path()
        manager.step()
        self.assertIs(manager.graph_search.planner.collision_filter.model, model)

    def test_stale_feedback_cancels_without_rrt(self):
        manager = self.manager()
        manager.step()
        self.now.return_value = 101.1
        manager.advance_graph_search()
        self.assertIsNone(manager.graph_search)
        self.assertEqual(manager.last_graph_search['reason'], 'FEEDBACK_STALE')
        self.assertEqual(manager.step().status, 'WAITING_FOR_FEEDBACK')
        self.assertFalse(manager.rrt_fallback_pending)

    def test_obstacle_update_cancels_the_old_scene(self):
        manager = self.manager()
        manager.step()
        snapshot = manager.graph_workspace.planner.collision_filter
        name = next(n for n in manager.planner.collision_filter.obstacle_geoms if n != 'target')
        old_sizes = snapshot.model.geom_size.copy()
        manager.update_obstacles([(name, [2., 2., 2.], [1., 0., 0., 0.], False)])
        self.assertIsNone(manager.graph_search)
        np.testing.assert_array_equal(snapshot.model.geom_size, old_sizes)
        self.assertFalse(manager.rrt_fallback_pending)

    def test_new_human_collision_cannot_execute_an_old_snapshot_route(self):
        manager = self.manager()
        manager.step()
        for _ in range(100000):
            manager.advance_graph_search()
            if manager.graph_search.done:
                break
        self.assertTrue(manager.graph_search.planned)
        goal = manager.graph_search.goal
        checker = manager.planner.collision_filter
        from benchmark_rrt_connect import geom_center
        from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
        center = geom_center(checker, goal, 'link4_visual')
        manager.update_human_snapshot(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 0, tuple(center), tuple(center + [0., 0., .08]), .05),)))
        self.assertFalse(checker.is_state_valid(goal))
        for _ in range(3):
            self.assertNotEqual(manager.step().action, 'EXECUTE')
        self.assertFalse(manager.rrt_fallback_pending)


if __name__ == '__main__':
    unittest.main()
