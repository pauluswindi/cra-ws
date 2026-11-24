"""Retreat dead ends, candidate priority and unchanged certification gates."""

import unittest

import numpy as np

from planning.path_planner import PathPlanner


class PlateauChecker:
    """Deterministic geometry control; native geometry is tested separately."""

    safety_margin = .005
    human_safety_margin = .05
    max_joint_step = .02

    def __init__(self, gain=.006, reject_final=False):
        self.environment_revision = 1
        self.ready = True
        self.gain = gain
        self.reject_final = reject_final
        self.edges = []
        self.on_state = None

    def human_data_ready(self):
        return self.ready

    def human_margin_state(self, q):
        if self.on_state:
            self.on_state(q)
        if not self.ready:
            return None
        clearance = .018 + np.clip(q[0] / .06, 0., 1.) * self.gain
        return dict(status='NEAR', clearance=float(clearance), pairs={(0, 1, 6): clearance})

    def diagnose_state(self, _):
        return dict(reason='HUMAN_DATA_UNKNOWN')

    def is_state_valid(self, _):
        return False

    def is_human_margin_transition_valid(self, a, b):
        self.edges.append((a.copy(), b.copy()))
        if self.reject_final and np.max(np.abs(b - a)) > .011:
            return False
        return self.ready


def planner_for(checker):
    planner = PathPlanner.__new__(PathPlanner)
    planner.collision_filter = checker
    planner.max_joint_delta = .8
    planner.max_segment_delta = .6
    return planner


class HumanRetreatSearchTests(unittest.TestCase):
    def test_dead_end_keeps_only_fully_certified_staged_progress(self):
        checker = PlateauChecker()
        planner = planner_for(checker)
        path = planner.find_human_margin_retreat(np.zeros(5))
        self.assertIsNotNone(path)
        np.testing.assert_allclose(path[-1], [.06, 0, 0, 0, 0])
        np.testing.assert_array_equal(checker.edges[-1][0], path[0])
        np.testing.assert_array_equal(checker.edges[-1][1], path[1])
        stats = planner.last_human_retreat_statistics
        self.assertEqual(stats['stop_condition'], 'NO_MONOTONIC_STEP')
        self.assertAlmostEqual(stats['gain_m'], .006)
        self.assertEqual(stats['reason'], 'NONE')
        self.assertTrue(stats['returned_path'])

    def test_dead_end_below_existing_minimum_gain_remains_blocked(self):
        planner = planner_for(PlateauChecker(gain=.004))
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5)))
        self.assertEqual(planner.last_human_retreat_failure, 'NO_MONOTONIC_STEP')

    def test_dead_end_cannot_bypass_straight_segment_certificate(self):
        planner = planner_for(PlateauChecker(reject_final=True))
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5)))
        self.assertFalse(planner.last_human_retreat_statistics['returned_path'])

    def test_expiry_during_endpoint_screening_rejects_existing_progress(self):
        checker = PlateauChecker()
        checker.on_state = lambda q: setattr(checker, 'ready', q[0] < .035)
        planner = planner_for(checker)
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5)))
        self.assertEqual(planner.last_human_retreat_failure, 'HUMAN_DATA_UNKNOWN')

    def test_revision_change_during_screening_rejects_existing_progress(self):
        checker = PlateauChecker()

        def change(q):
            if q[0] > .035:
                checker.environment_revision = 2

        checker.on_state = change
        planner = planner_for(checker)
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5)))
        self.assertEqual(planner.last_human_retreat_failure, 'ENVIRONMENT_CHANGED')

    def test_step_limit_is_distinguished_from_wall_time_budget(self):
        planner = planner_for(PlateauChecker())
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5), max_steps=2))
        stats = planner.last_human_retreat_statistics
        self.assertEqual(stats['stop_condition'], 'STEP_LIMIT')
        self.assertEqual(stats['steps'], 2)
        self.assertEqual(stats['reason'], 'SEARCH_BUDGET_EXHAUSTED')
        self.assertGreaterEqual(stats['elapsed_ms'], 0.)

    def test_best_gain_first_skips_lower_ranked_transition_queries(self):
        checker = PlateauChecker()

        def states(q):
            clearance = .018 + float(q @ np.array([.01, .02, .03, .04, .05]))
            return dict(status='NEAR', clearance=clearance, pairs={(0, 1, 6): clearance})

        checker.human_margin_state = states
        planner = planner_for(checker)
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5), max_steps=1))
        self.assertEqual(len(checker.edges), 1)
        np.testing.assert_array_equal(checker.edges[0][1], [0, 0, 0, 0, .01])

    def test_invalid_top_ranked_transition_does_not_hide_valid_runner_up(self):
        checker = PlateauChecker()
        checker.human_margin_state = lambda q: dict(
            status='NEAR', clearance=.018 + .1 * q[0] + .2 * q[1],
            pairs={(0, 1, 6): .018 + .1 * q[0] + .2 * q[1]})
        original = checker.is_human_margin_transition_valid
        checker.is_human_margin_transition_valid = lambda a, b: (
            original(a, b) and b[1] == 0.)
        planner = planner_for(checker)
        planner.find_human_margin_retreat(np.zeros(5), max_steps=1)
        self.assertEqual(len(checker.edges), 2)
        np.testing.assert_array_equal(checker.edges[0][1], [0, .01, 0, 0, 0])
        np.testing.assert_array_equal(checker.edges[1][1], [.01, 0, 0, 0, 0])

    def test_equal_gains_preserve_joint_sign_tie_order(self):
        checker = PlateauChecker()
        checker.human_margin_state = lambda q: dict(
            status='NEAR', clearance=.018 + .1 * np.abs(q).sum(),
            pairs={(0, 1, 6): .018 + .1 * np.abs(q).sum()})
        planner_for(checker).find_human_margin_retreat(np.zeros(5), max_steps=1)
        np.testing.assert_array_equal(checker.edges[0][1], [-.01, 0, 0, 0, 0])

    def test_near_endpoints_do_not_permit_worsening_another_pair(self):
        checker = PlateauChecker()
        checker.human_margin_state = lambda q: dict(
            status='NEAR', clearance=.018 + .1 * q[0],
            pairs={(0, 1, 6): .018 + .1 * q[0], (1, 1, 6): .03 - .001 * q[0]})
        planner = planner_for(checker)
        self.assertIsNone(planner.find_human_margin_retreat(np.zeros(5)))
        self.assertEqual(checker.edges, [])


if __name__ == '__main__':
    unittest.main()
