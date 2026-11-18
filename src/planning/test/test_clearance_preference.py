"""Verify soft preferences never relax mandatory geometry or final guards."""

import unittest
from unittest.mock import patch

import numpy as np

from planning.algorithms.clearance_preference import ClearancePathPreference, _PreferredChecker
from planning.algorithms.rrt_connect import RRTConnectPlanner
from test_rrt_connect import WallChecker, q


class HumanWallChecker(WallChecker):
    def __init__(self):
        super().__init__()
        self.human_obstacles_enabled = True
        self.human_safety_margin = .05
        self.max_joint_step = .02
        self.on_report = None

    def clearance(self, point):
        return float(np.hypot(max(abs(point[0]) - .15, 0.),
                              max(point[1] - self.ceiling, 0.)))

    def is_state_valid(self, point):
        return super().is_state_valid(point) and self.clearance(point) > .05

    def human_collision_report(self, point):
        if self.on_report:
            self.on_report()
        clearance = self.clearance(point)
        return {'status': 'FAR' if clearance > .05 else 'NEAR', 'clearance': clearance}


SEARCH = 'planning.algorithms.clearance_preference.RRTConnectPlanner.plan'


class ClearancePreferenceTests(unittest.TestCase):
    def setUp(self):
        self.checker = HumanWallChecker()
        self.baseline = np.array([q(-.8, -.6), q(-.3, .36), q(.3, .36), q(.8, -.6)])
        self.clearer = np.array([q(-.8, -.6), q(-.3, .5), q(.3, .5), q(.8, -.6)])
        self.selector = ClearancePathPreference(self.checker, clock=lambda: 0.)

    def test_selects_clearer_path_with_bounded_added_joint_travel(self):
        original = self.baseline.copy()
        with patch(SEARCH, return_value=self.clearer):
            result = self.selector.improve(self.baseline)
        np.testing.assert_array_equal(result, self.clearer)
        np.testing.assert_array_equal(self.baseline, original)
        stats = self.selector.last_statistics
        self.assertEqual(stats['selection'], 'CLEARER_PATH_SELECTED')
        self.assertGreater(stats['selected_score']['min_clearance_m'], .19)
        self.assertLessEqual(stats['selected_score']['length_rad'],
                             1.25 * stats['baseline_score']['length_rad'])
        self.assertEqual(self.checker.human_safety_margin, .05)

    def test_measures_edge_bottleneck_not_only_far_waypoints(self):
        with patch(SEARCH, return_value=None):
            self.selector.improve(self.baseline)
        score = self.selector.last_statistics['baseline_score']
        self.assertAlmostEqual(score['min_clearance_m'], .06)
        self.assertGreater(min(self.checker.clearance(p) for p in self.baseline), .15)

    def test_excessively_long_clearer_path_keeps_certified_baseline(self):
        long_path = np.array([self.baseline[0], q(-.95, .95), q(.95, .95), self.baseline[-1]])
        with patch(SEARCH, return_value=long_path):
            result = self.selector.improve(self.baseline)
        np.testing.assert_array_equal(result, self.baseline)
        self.assertEqual(self.selector.last_statistics['candidate_reason'], 'LENGTH_CAP_EXCEEDED')

    def test_preference_failure_keeps_baseline_instead_of_reporting_no_route(self):
        with patch(SEARCH, return_value=None):
            result = self.selector.improve(self.baseline)
        np.testing.assert_array_equal(result, self.baseline)
        self.assertEqual(self.selector.last_plan_failure, 'NONE')

    def test_fixed_near_endpoint_caps_soft_band_without_changing_hard_margin(self):
        path = np.array([q(-.21, 0.), q(-.8, .6), q(.8, .6), q(.8, -.6)])
        result = self.selector.improve(path)
        np.testing.assert_array_equal(result, path)
        self.assertEqual(
            self.selector.last_statistics['candidate_reason'], 'PREFERENCE_ALREADY_MET')
        self.assertAlmostEqual(self.selector.last_statistics['effective_preference_m'], .06 - 1e-6)
        self.assertEqual(self.checker.human_safety_margin, .05)

    def test_unnecessarily_higher_clearance_does_not_force_extra_search(self):
        with patch(SEARCH) as search:
            result = self.selector.improve(self.clearer)
        np.testing.assert_array_equal(result, self.clearer)
        search.assert_not_called()
        self.assertEqual(
            self.selector.last_statistics['candidate_reason'], 'PREFERENCE_ALREADY_MET')

    def test_candidate_crossing_wall_cannot_replace_valid_baseline(self):
        with patch(SEARCH, return_value=self.baseline[[0, -1]]):
            result = self.selector.improve(self.baseline)
        np.testing.assert_array_equal(result, self.baseline)
        self.assertEqual(self.selector.last_statistics['candidate_reason'], 'CANDIDATE_INVALID')

    def test_candidate_cannot_change_the_requested_start_or_goal(self):
        wrong_goal = self.clearer.copy()
        wrong_goal[-1] = q(.9, -.6)
        with patch(SEARCH, return_value=wrong_goal):
            result = self.selector.improve(self.baseline)
        np.testing.assert_array_equal(result, self.baseline)
        self.assertEqual(self.selector.last_statistics['candidate_reason'], 'ENDPOINT_MISMATCH')

    def test_invalid_baseline_is_never_rescued_by_skipping_checks(self):
        self.assertIsNone(self.selector.improve(self.baseline[[0, -1]]))
        self.assertEqual(self.selector.last_plan_failure, 'BASELINE_INVALID')

    def test_revision_change_during_scoring_rejects_every_candidate(self):
        self.checker.on_report = lambda: setattr(self.checker, 'environment_revision', 1)
        self.assertIsNone(self.selector.improve(self.baseline))
        self.assertEqual(self.selector.last_plan_failure, 'ENVIRONMENT_CHANGED')

    def test_expiry_during_final_validation_does_not_return_cached_baseline(self):
        self.checker.on_edge = lambda: setattr(self.checker, 'ready', self.checker.edges < 4)
        with patch(SEARCH, return_value=None):
            self.assertIsNone(self.selector.improve(self.baseline))
        self.assertEqual(self.selector.last_plan_failure, 'HUMAN_VISION_UNKNOWN')

    def test_budget_expiry_during_final_validation_rejects_result(self):
        clock = [0.]
        selector = ClearancePathPreference(self.checker, clock=lambda: clock[0])

        def expire():
            if self.checker.edges == 4:
                clock[0] = 3.

        self.checker.on_edge = expire
        with patch(SEARCH, return_value=None):
            self.assertIsNone(selector.improve(self.baseline))
        self.assertEqual(selector.last_plan_failure, 'TIME_BUDGET_EXHAUSTED')

    def test_checker_exception_returns_no_path(self):
        def fail():
            raise RuntimeError('distance failure')

        self.checker.on_report = fail
        self.assertIsNone(self.selector.improve(self.baseline))
        self.assertEqual(self.selector.last_plan_failure, 'COLLISION_CHECK_ERROR')

    def test_invalid_and_stationary_inputs_rejected(self):
        for path in (None, [q(0.)], np.array([q(0.), q(0.)]), [[float('nan')] * 5] * 2):
            self.assertIsNone(self.selector.improve(path))
            self.assertEqual(self.selector.last_plan_failure, 'INVALID_INPUT')

    def test_configuration_cannot_lower_margin_or_allow_unbounded_length(self):
        for options in ({'preferred_clearance': .04}, {'max_length_ratio': .9},
                        {'search_budget': 0.}, {'time_budget': float('inf')}):
            with self.assertRaises(ValueError):
                ClearancePathPreference(self.checker, **options)


class NativePreferredCheckerTests(unittest.TestCase):
    def setUp(self):
        from benchmark_rrt_connect import make_scenes, planner

        clock = patch('time.monotonic', lambda: 100.)
        clock.start()
        self.addCleanup(clock.stop)
        self.scene = make_scenes()[1]
        self.checker = planner(self.scene[1]).collision_filter

    def test_native_search_uses_separate_state_cache_and_stricter_preference(self):
        checker = self.checker
        start, goal = self.scene[2:]
        self.assertTrue(checker.is_state_valid(start))
        initial = checker.data.qpos.copy()
        refined = checker._human_collision_after_forward(include_pairs=True)['clearance']
        proxy = _PreferredChecker(checker, refined + .01, .01)
        self.assertFalse(proxy.is_state_valid(start))
        proxy.is_state_valid(goal)
        np.testing.assert_array_equal(checker.data.qpos, initial)
        np.testing.assert_array_equal(proxy.search_checker.data.mocap_pos, checker.data.mocap_pos)
        self.assertIsNot(proxy.search_checker._human_report_cache, checker._human_report_cache)
        self.assertEqual(checker.human_safety_margin, .05)
        self.assertEqual(proxy.search_checker.human_safety_margin, .05)
        self.assertTrue(checker.is_state_valid(start))

    def test_native_snapshot_cannot_borrow_a_new_observation(self):
        from test_dynamic_replanning import FAR

        proxy = _PreferredChecker(self.checker, .06, .01)
        self.checker.set_human_capsules(FAR)
        self.assertFalse(proxy.human_data_ready())
        search = RRTConnectPlanner(proxy)
        self.assertIsNone(search.plan(self.scene[2], self.scene[3]))
        self.assertEqual(search.last_plan_failure, 'HUMAN_VISION_UNKNOWN')


if __name__ == '__main__':
    unittest.main()
