"""Local projection evidence must preserve ordering and reject ambiguity."""

import unittest
import json
from pathlib import Path
from types import MethodType, SimpleNamespace

import numpy as np

from planning.execution_progress import OrderedSegmentProgress
from planning.planning_manager import PlanningManager


def q(x, y=0.):
    return np.array([x, y, 0., 0., 0.])


class OrderedSegmentProgressTests(unittest.TestCase):
    def setUp(self):
        self.path = np.array([q(0.), q(.2), q(.2, .2), q(.4, .2)])

    def test_missed_corner_requires_two_forward_samples_on_the_adjacent_segment(self):
        tracker = OrderedSegmentProgress(self.path, corridor_tolerance=.04)
        tracker.observe(q(.16, .03), q(.185, .03))
        tracker.observe(q(.185, .03), q(.2, .035))
        tracker.observe(q(.2, .035), q(.2, .045))
        self.assertEqual(tracker.next_waypoint, 1)
        self.assertEqual(tracker.confirmed_samples, 1)
        tracker.observe(q(.2, .045), q(.2, .055))
        self.assertEqual(tracker.next_waypoint, 2)
        self.assertEqual(tracker.reason, 'CANDIDATE_NEXT_SEGMENT')

    def test_stationary_duplicate_and_backward_noise_do_not_confirm_a_transition(self):
        tracker = OrderedSegmentProgress(self.path, corridor_tolerance=.04)
        tracker.observe(q(.16, .03), q(.185, .03))
        tracker.observe(q(.185, .03), q(.2, .045))
        tracker.observe(q(.2, .045), q(.2, .045))
        self.assertEqual(tracker.confirmed_samples, 0)
        tracker.observe(q(.2, .045), q(.2, .044))
        self.assertEqual(tracker.next_waypoint, 1)
        tracker.observe(q(.2, .044), q(.2, .046))
        self.assertEqual(tracker.next_waypoint, 1)

    def test_future_loop_proximity_is_not_global_progress(self):
        path = np.array([q(0.), q(.2), q(.2, .2), q(0., .2), q(0.), q(-.2)])
        tracker = OrderedSegmentProgress(path, corridor_tolerance=.04)
        tracker.observe(q(0.), q(0., .01))
        tracker.observe(q(0., .01), q(0., .02))
        self.assertEqual(tracker.next_waypoint, 1)
        tracker.observe(q(.16, .03), q(.185, .03))
        tracker.observe(q(.185, .03), q(0., .2))
        self.assertEqual(tracker.next_waypoint, 1)
        self.assertFalse(tracker.armed)

    def test_outside_corridor_and_feedback_jump_discard_approach_evidence(self):
        for destination in (q(.15, .06), q(.2, .18)):
            with self.subTest(destination=destination):
                tracker = OrderedSegmentProgress(self.path, corridor_tolerance=.04)
                tracker.observe(q(.16, .03), q(.185, .03))
                tracker.observe(q(.185, .03), destination)
                self.assertEqual(tracker.next_waypoint, 1)
                self.assertFalse(tracker.armed)

    def test_overlapping_return_segment_remains_conservative_without_direct_passage(self):
        tracker = OrderedSegmentProgress(
            np.array([q(0.), q(.2), q(0.), q(-.2)]), corridor_tolerance=.04)
        tracker.observe(q(.16, .03), q(.185, .03))
        tracker.observe(q(.185, .03), q(.17, .03))
        tracker.observe(q(.17, .03), q(.15, .03))
        self.assertEqual(tracker.next_waypoint, 1)

    def test_invalid_configuration_and_feedback_cannot_advance(self):
        for settings in ({'corridor_tolerance': 0.}, {'confirmation_samples': 1},
                         {'confirmation_samples': 2.5}, {'handoff_fraction': 1.},
                         {'max_feedback_step': float('nan')}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                OrderedSegmentProgress(self.path, **settings)
        tracker = OrderedSegmentProgress(self.path)
        tracker.observe(q(0.), np.full(5, np.nan))
        self.assertEqual(tracker.next_waypoint, 1)
        self.assertFalse(tracker.armed)

    def test_recorded_missed_corner_chain_reaches_final_segment_without_global_search(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/recorded_progress_235941.json')
                             .read_text())
        path = np.array(fixture['path'])
        results = []
        for enabled in (False, True):
            holder = SimpleNamespace(
                last_execution_path=path, execution_next_waypoint=1,
                execution_progress_reason='COMMAND_SENT',
                planner=SimpleNamespace(joint_goal_tolerance=.02),
                _stop_required=lambda: True)
            tracker = OrderedSegmentProgress(path, corridor_tolerance=.04)
            holder._advance_direct_waypoints = MethodType(
                PlanningManager._advance_direct_waypoints, holder)
            previous = path[0].copy()
            for values in fixture['feedback']:
                current = np.array(values)
                holder._advance_direct_waypoints(previous, current)
                if enabled:
                    tracker.synchronize(holder.execution_next_waypoint)
                    tracker.observe(previous, current)
                    # Estimator-only replay; real collision authorization has
                    # separate manager tests, and no Motion action is emitted.
                    holder.execution_next_waypoint = tracker.next_waypoint
                previous = current
            results.append(holder.execution_next_waypoint)
        self.assertEqual(results, [5, 13])


if __name__ == '__main__':
    unittest.main()
