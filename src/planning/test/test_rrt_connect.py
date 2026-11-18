"""Check search geometry and bounded failure without ROS or robot commands."""

import unittest

import numpy as np

from planning.algorithms.rrt_connect import RRTConnectPlanner


def q(x, y=0.):
    return np.array([x, y, 0., 0., 0.])


class WallChecker:
    """Exact slab intersection in two dimensions, embedded in five joints."""

    def __init__(self, ceiling=.3):
        self.lower = np.full(5, -1.)
        self.upper = np.full(5, 1.)
        self.environment_revision = 0
        self.human_obstacles_enabled = False
        self.ready = True
        self.ceiling = ceiling
        self.on_edge = None
        self.edges = 0
        self.directed_certificates = set()

    def human_data_ready(self):
        return self.ready

    def is_state_valid(self, point):
        return (np.all(point >= self.lower) and np.all(point <= self.upper)
                and not (-.15 <= point[0] <= .15 and point[1] <= self.ceiling))

    def segment_clear(self, start, end):
        low, high = 0., 1.
        for index, limits in enumerate([(-.15, .15), (-1., self.ceiling)]):
            delta = end[index] - start[index]
            if abs(delta) < 1e-14:
                if not limits[0] <= start[index] <= limits[1]:
                    return True
                continue
            left, right = sorted((limit - start[index]) / delta for limit in limits)
            low, high = max(low, left), min(high, right)
            if low > high:
                return True
        return False

    def is_edge_valid(self, start, end):
        self.edges += 1
        if self.on_edge:
            self.on_edge()
        valid = (self.is_state_valid(start) and self.is_state_valid(end)
                 and self.segment_clear(start, end))
        if valid:
            self.directed_certificates.add((tuple(start), tuple(end)))
        return valid


class RRTConnectTests(unittest.TestCase):
    def setUp(self):
        self.checker = WallChecker()
        self.start, self.goal = q(-.8, -.6), q(.8, -.6)

    def planner(self, **kwargs):
        return RRTConnectPlanner(self.checker, clock=lambda: 0., **kwargs)

    def assert_path(self, path, start=None, goal=None):
        self.assertIsNotNone(path)
        np.testing.assert_array_equal(path[0], self.start if start is None else start)
        np.testing.assert_array_equal(path[-1], self.goal if goal is None else goal)
        self.assertTrue(all(self.checker.is_state_valid(point) for point in path))
        for a, b in zip(path[:-1], path[1:]):
            self.assertTrue(self.checker.segment_clear(a, b))
            self.assertIn((tuple(a), tuple(b)), self.checker.directed_certificates)

    def test_valid_endpoints_do_not_allow_crossing_wall(self):
        self.assertFalse(self.checker.segment_clear(self.start, self.goal))
        for seed in range(4):
            planner = self.planner()
            path = planner.plan(self.start, self.goal, seed=seed)
            self.assert_path(path)
            self.assertGreater(path[:, 1].max(), self.checker.ceiling)
            self.assertGreater(len(path), 2)
            self.assertLessEqual(np.linalg.norm(np.diff(path, axis=0), axis=1).max(), .15 + 1e-12)
            self.assertEqual(planner.last_plan_failure, 'NONE')

    def test_direct_free_path_does_not_grow_random_trees(self):
        start, goal = q(-.8, .7), q(.8, .7)
        planner = self.planner()
        path = planner.plan(start, goal)
        self.assert_path(path, start, goal)
        self.assertEqual(len(path), 2)
        self.assertEqual(planner.last_statistics['iterations'], 0)

    def test_fixed_seed_reproduces_both_tree_branches_without_input_mutation(self):
        planner = self.planner()
        original = self.start.copy()
        first = planner.plan(self.start, self.goal, seed=9)
        np.testing.assert_array_equal(first, planner.plan(self.start, self.goal, seed=9))
        np.testing.assert_array_equal(self.start, original)
        first[0] = 100.
        self.assert_path(planner.plan(self.start, self.goal, seed=9))

    def test_full_wall_returns_iteration_limit_not_false_no_path_proof(self):
        self.checker.ceiling = 1.
        planner = self.planner(max_iterations=20)
        self.assertIsNone(planner.plan(self.start, self.goal))
        self.assertEqual(planner.last_plan_failure, 'ITERATION_LIMIT')

    def test_invalid_start_is_not_used_as_margin_escape(self):
        planner = self.planner()
        self.assertIsNone(planner.plan(q(0.), self.goal))
        self.assertEqual(planner.last_plan_failure, 'START_INVALID')
        self.assertEqual(self.checker.edges, 0)

    def test_invalid_goal_is_rejected_before_tree_growth(self):
        planner = self.planner()
        self.assertIsNone(planner.plan(self.start, q(0.)))
        self.assertEqual(planner.last_plan_failure, 'GOAL_INVALID')

    def test_invalid_inputs_are_rejected_without_checker_calls(self):
        planner = self.planner()
        for start in ([0., 0.], [float('nan')] * 5, q(-2.)):
            self.assertIsNone(planner.plan(start, self.goal))
            self.assertEqual(planner.last_plan_failure, 'INVALID_INPUT')
            self.assertEqual(planner.last_statistics['state_checks'], 0)
        self.assertIsNone(planner.plan(self.start, self.goal, seed=-1))

    def test_same_valid_configuration_is_stationary_not_motion(self):
        planner = self.planner()
        path = planner.plan(self.start, self.start)
        self.assertEqual(path.shape, (1, 5))
        self.assertEqual(planner.last_statistics['path_length_rad'], 0.)

    def test_no_human_data_rejects_search(self):
        self.checker.human_obstacles_enabled, self.checker.ready = True, False
        planner = self.planner()
        self.assertIsNone(planner.plan(self.start, self.goal))
        self.assertEqual(planner.last_plan_failure, 'HUMAN_VISION_UNKNOWN')
        self.assertEqual(planner.last_statistics['state_checks'], 0)

    def test_expiry_during_successful_edge_cannot_return_or_poison_next_request(self):
        self.checker.human_obstacles_enabled = True
        self.checker.on_edge = lambda: setattr(self.checker, 'ready', False)
        planner = self.planner()
        self.assertIsNone(planner.plan(q(-.8, .7), q(.8, .7)))
        self.assertEqual(planner.last_plan_failure, 'HUMAN_VISION_UNKNOWN')
        self.checker.on_edge, self.checker.ready = None, True
        self.assert_path(planner.plan(self.start, self.goal))

    def test_revision_change_inside_edge_cancels_request(self):
        self.checker.on_edge = lambda: setattr(self.checker, 'environment_revision', 1)
        planner = self.planner()
        self.assertIsNone(planner.plan(self.start, self.goal))
        self.assertEqual(planner.last_plan_failure, 'ENVIRONMENT_CHANGED')

    def test_expiry_during_final_certification_rejects_already_connected_path(self):
        self.checker.human_obstacles_enabled = True

        def expire():
            if self.checker.edges == 2:
                self.checker.ready = False

        self.checker.on_edge = expire
        planner = self.planner()
        self.assertIsNone(planner.plan(q(-.8, .7), q(.8, .7)))
        self.assertEqual(planner.last_plan_failure, 'HUMAN_VISION_UNKNOWN')
        self.assertEqual(planner.last_statistics['waypoints'], 0)

    def test_time_budget_includes_checker_call_and_final_validation(self):
        for expire_on in (1, 2):
            with self.subTest(expire_on=expire_on):
                self.checker = WallChecker()
                clock = [0.]

                def advance():
                    if self.checker.edges == expire_on:
                        clock[0] = .6

                self.checker.on_edge = advance
                planner = RRTConnectPlanner(self.checker, clock=lambda: clock[0])
                self.assertIsNone(planner.plan(q(-.8, .7), q(.8, .7)))
                self.assertEqual(planner.last_plan_failure, 'TIME_BUDGET_EXHAUSTED')

    def test_global_node_limit_bounds_connect_loop_too(self):
        planner = self.planner(max_nodes=3)
        self.assertIsNone(planner.plan(self.start, self.goal))
        self.assertEqual(planner.last_plan_failure, 'NODE_LIMIT')
        self.assertEqual(planner.last_statistics['nodes'], 3)

    def test_checker_exception_has_explicit_failure_without_partial_path(self):
        def fail():
            raise RuntimeError('checker probe')

        self.checker.on_edge = fail
        planner = self.planner()
        self.assertIsNone(planner.plan(self.start, self.goal))
        self.assertEqual(planner.last_plan_failure, 'COLLISION_CHECK_ERROR')
        self.assertIn('checker probe', planner.last_error)

    def test_configuration_validation(self):
        for options in ({'step_size': 0.}, {'time_budget': float('inf')},
                        {'max_nodes': 1}, {'max_iterations': True}):
            with self.assertRaises(ValueError):
                self.planner(**options)
        self.checker.lower[0] = float('-inf')
        with self.assertRaises(ValueError):
            self.planner()


if __name__ == '__main__':
    unittest.main()
