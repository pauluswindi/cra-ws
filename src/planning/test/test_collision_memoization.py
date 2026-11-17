"""Verify exact human geometry reuse without relaxing freshness or contacts."""

from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from planning.algorithms.collision_filter import CollisionFilter
from planning.algorithms.human_collision import nearest_human_collision
from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from simulation.frame_transform import FrameTransform


ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml'
JOINTS = [f'joint{i}' for i in range(1, 6)]
START = np.array([.023162707686424255, -1.3472504615783691,
                  1.1848934888839722, -.5477003455162048, .6865991950035095])
FAR = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
    HumanCapsule(1, 0, (10., 10., 10.), (10., 10., 11.), .05),))
QUERY = 'planning.algorithms.collision_filter.nearest_human_collision'


class HumanCollisionMemoizationTests(unittest.TestCase):
    def setUp(self):
        clock = patch('time.monotonic', return_value=100.)
        self.now = clock.start()
        self.addCleanup(clock.stop)
        self.checker = self.make_checker()

    @staticmethod
    def make_checker(capacity=256):
        checker = CollisionFilter(MODEL, JOINTS, human_data_timeout=2.,
                                  human_report_cache_size=capacity)
        checker.enable_human_obstacles(True)
        checker.set_human_capsules(FAR)
        return checker

    def test_exact_repeat_reuses_geometry_but_still_runs_forward_kinematics(self):
        c = self.checker
        with patch(QUERY, wraps=nearest_human_collision) as query, \
                patch.object(c, '_set_joint_configuration',
                             wraps=c._set_joint_configuration) as forward:
            first = c.human_collision_report(START)
            self.assertEqual(first, c.human_collision_report(START.copy()))
            self.assertEqual(query.call_count, 1)
            self.assertEqual(forward.call_count, 2)
        self.assertEqual(c.human_report_cache_hits, 1)

    def test_expired_receipt_cannot_reuse_a_clear_result(self):
        c = self.checker
        self.assertEqual(c.human_collision_report(START)['status'], 'FAR')
        self.now.return_value = 102.001
        self.assertEqual(c.human_collision_report(START)['status'], 'UNKNOWN')
        self.assertFalse(c.is_state_valid(START))

    def test_capture_expiry_wins_over_the_longer_receipt_budget(self):
        c = self.checker
        c.set_human_capsules(HumanSnapshot(
            'OBSERVED', 'CAPSULES_PRESENT', FAR.capsules, expires_at=100.1))
        self.assertEqual(c.human_collision_report(START)['status'], 'FAR')
        self.now.return_value = 100.2
        self.assertEqual(c.human_collision_report(START)['status'], 'UNKNOWN')

    def test_expiry_during_a_cache_hit_or_full_query_is_rejected(self):
        c = self.checker
        c.human_collision_report(START)
        for capacity in (256, 0):
            c.human_report_cache_size = capacity
            with self.subTest(capacity=capacity), \
                    patch.object(c, 'human_data_ready', side_effect=[True, True, False]):
                self.assertEqual(c.human_collision_report(START)['status'], 'UNKNOWN')

    def test_a_new_identical_observation_discards_previous_geometry(self):
        c = self.checker
        c.human_collision_report(START)
        revision = c.environment_revision
        self.now.return_value = 100.2
        c.set_human_capsules(FAR)
        self.assertEqual(c.environment_revision, revision)
        self.assertEqual(len(c._human_report_cache), 0)
        with patch(QUERY, wraps=nearest_human_collision) as query:
            c.human_collision_report(START)
            self.assertEqual(query.call_count, 1)

    def test_a_moved_human_does_not_inherit_a_clear_result(self):
        c = self.checker
        self.assertEqual(c.human_collision_report(START)['status'], 'FAR')
        geom, (center, _) = next(iter(c.robot_human_boxes.items()))
        rotation = c.data.geom_xmat[geom].reshape(3, 3)
        world = c.data.geom_xpos[geom] + rotation @ center
        base = c.data.xmat[c.base_id].reshape(3, 3).T @ (world - c.data.xpos[c.base_id])
        point = tuple(FrameTransform.mujoco_base_to_robot_base(base))
        c.set_human_capsules(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 0, point, point, .05),)))
        self.assertEqual(c.human_collision_report(START)['status'], 'NEAR')

    def test_static_contact_is_checked_even_after_a_human_cache_hit(self):
        c = self.checker
        self.assertTrue(c.is_state_valid(START))
        self.assertTrue(c.is_state_valid(START))
        self.assertGreater(c.human_report_cache_hits, 0)
        geom = next(iter(c.robot_human_boxes))
        position = c.data.geom_xpos[geom].copy()
        c.set_obstacle_pose('obstacle_1', position, size=[.2, .2, .2])
        self.assertEqual(len(c._human_report_cache), 0)
        self.assertFalse(c.is_state_valid(START))
        self.assertEqual(c.last_state_validation_failure['reason'], 'MUJOCO_CONTACT')

    def test_diagnostic_mutation_does_not_change_cached_retreat_pairs(self):
        c = self.checker
        c._set_joint_configuration(START)
        first = c._human_collision_after_forward(include_pairs=True)
        expected = first['pairs'].copy()
        first['clearance'] = -100.
        first['pairs'].clear()
        second = c._human_collision_after_forward(include_pairs=True)
        self.assertGreater(second['clearance'], 0.)
        self.assertEqual(second['pairs'], expected)
        second['pairs'].clear()
        self.assertEqual(c._human_collision_after_forward(True)['pairs'], expected)

    def test_detailed_pairs_do_not_reuse_a_sphere_only_report(self):
        c = self.checker
        c.human_collision_report(START)
        detailed = c._human_collision_after_forward(True)
        self.assertIn('pairs', detailed)
        self.assertEqual(c.human_report_cache_hits, 0)

    def test_margin_and_mesh_box_changes_invalidate_exact_input_keys(self):
        c = self.checker
        c._set_joint_configuration(START)
        c._human_collision_after_forward(True)
        c.human_safety_margin = .12
        updated = c._human_collision_after_forward(True)
        self.assertEqual(updated['safety_margin'], .12)
        self.assertEqual(c.human_report_cache_hits, 0)
        c.robot_human_boxes = {}
        updated = c._human_collision_after_forward(True)
        self.assertEqual(updated['geometry_model'], 'SPHERE')
        self.assertEqual(c.human_report_cache_hits, 0)

    def test_cache_is_bounded_and_can_be_disabled(self):
        for capacity in (0, 2):
            c = self.make_checker(capacity)
            with patch(QUERY, wraps=nearest_human_collision) as query:
                for delta in (0., .01, .02, 0.):
                    q = START.copy()
                    q[0] += delta
                    c.human_collision_report(q)
                self.assertEqual(query.call_count, 4)
            self.assertLessEqual(len(c._human_report_cache), capacity)

    def test_random_cached_reports_match_full_geometry_in_both_report_modes(self):
        cached, full = self.checker, self.make_checker(0)
        snapshot = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 0, (.2, -.15, -.44), (.2, -.15, -.36), .03),
            HumanCapsule(1, 6, (.1, -.1, -.3), (.1, -.1, .3), .05),))
        cached.set_human_capsules(snapshot)
        full.set_human_capsules(snapshot)
        rng = np.random.default_rng(74)
        for q in rng.uniform(cached.lower, cached.upper, size=(30, 5)):
            for detailed in (False, True):
                cached._set_joint_configuration(q)
                full._set_joint_configuration(q)
                reference = full._human_collision_after_forward(detailed)
                self.assertEqual(cached._human_collision_after_forward(detailed), reference)
                self.assertEqual(cached._human_collision_after_forward(detailed), reference)
        self.assertEqual(cached.human_report_cache_hits, 60)

    def test_invalid_cache_size_is_rejected(self):
        for capacity in (-1, True, 1.5, None):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                self.make_checker(capacity)


if __name__ == '__main__':
    unittest.main()
