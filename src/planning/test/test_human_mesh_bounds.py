"""Verify conservative mesh boxes and finite capsule distances with real geometry."""

import contextlib
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from planning.algorithms.human_collision import (
    nearest_human_collision, segment_box_distances,
)
from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from planning.path_planner import PathPlanner
from simulation.frame_transform import FrameTransform


ROOT = Path(__file__).resolve().parents[3]


class SegmentBoxTests(unittest.TestCase):
    def test_face_corner_intersection_parallel_and_point_segments(self):
        cases = [
            ((2, 0, 0), (3, 0, 0), 1.),
            ((2, 2, 2), (3, 3, 3), np.sqrt(3)),
            ((-2, 0, 0), (2, 0, 0), 0.),
            ((-2, 1, 0), (2, 1, 0), 0.),
            ((-2, 2, 0), (2, 2, 0), 1.),
            ((2, 0, 0), (2, 0, 0), 1.),
            ((0, 0, 0), (0, 0, 0), 0.),
            ((-2, 1.1, 0), (2, 1.1 + 1e-14, 0), .1),
        ]
        for start, end, expected in cases:
            with self.subTest(start=start, end=end):
                self.assertAlmostEqual(segment_box_distances(start, end, (1, 1, 1)),
                                       expected, places=12)
                self.assertAlmostEqual(segment_box_distances(end, start, (1, 1, 1)),
                                       expected, places=12)

    def test_random_batches_match_independent_convex_scalar_minimization(self):
        rng = np.random.default_rng(98)
        starts, ends = rng.normal(size=(2, 300, 3)) * 2.
        halves = rng.uniform(.01, 1.5, size=(300, 3))
        actual = segment_box_distances(starts, ends, halves)
        expected = []
        for start, end, half in zip(starts, ends, halves):
            delta = end - start
            left, right = 0., 1.
            # The derivative of squared point/box distance is monotonic.
            for _ in range(80):
                middle = (left + right) * .5
                point = start + middle * delta
                derivative = float((point - np.clip(point, -half, half)) @ delta)
                if derivative < 0:
                    left = middle
                else:
                    right = middle
            point = start + ((left + right) * .5) * delta
            expected.append(np.linalg.norm(point - np.clip(point, -half, half)))
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-12)

    def test_rotated_box_and_frame_conversion_use_the_full_finite_segment(self):
        angle = .43
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                             [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        center = np.array([.3, -.6, .4])
        conversion = FrameTransform.ROBOT_FROM_MUJOCO_BASE
        # Endpoints are both outside, but the middle crosses the box.
        start = conversion @ (center + rotation @ [-2., 0., 0.])
        end = conversion @ (center + rotation @ [2., 0., 0.])
        report = nearest_human_collision(
            [('test', center, 2.)], [((1, 6), start, end, .05)], .1,
            robot_boxes=[(center, rotation, [.1, .2, .3])], include_pairs=True)
        self.assertEqual(report['status'], 'NEAR')
        self.assertAlmostEqual(report['box_clearance'], -.05)
        self.assertAlmostEqual(report['clearance'], -.05)
        self.assertEqual(report['geometry_model'], 'SPHERE_AND_MESH_BOX')
        self.assertAlmostEqual(report['pairs'][(0, 1, 6)], -.05)

    def test_invalid_potential_collision_box_remains_unknown(self):
        geometry = [('test', [0, 0, 0], 1.)]
        capsules = [((1, 6), (0, 0, 0), (0, 0, 1), .05)]
        for box in [([0, 0, 0], np.eye(3) * 2., [1, 1, 1]),
                    ([0, 0, np.nan], np.eye(3), [1, 1, 1]),
                    ([0, 0, 0], np.eye(3), [-1, 1, 1])]:
            report = nearest_human_collision(geometry, capsules, .1, robot_boxes=[box])
            self.assertEqual(report['reason'], 'INVALID_ROBOT_BOX')

    def test_ten_centimeter_margin_is_preserved_at_the_refined_surface(self):
        geometry = [('test', [0., 0., 0.], 1.)]
        box = [(np.zeros(3), np.eye(3), [.1, .2, .3])]
        conversion = FrameTransform.ROBOT_FROM_MUJOCO_BASE
        for clearance, status in [(.0999, 'NEAR'), (.1001, 'FAR')]:
            point = conversion @ [.1 + .05 + clearance, 0., 0.]
            report = nearest_human_collision(
                geometry, [((1, 6), point, point, .05)], .1, robot_boxes=box)
            self.assertEqual(report['status'], status)
            self.assertAlmostEqual(report['clearance'], clearance, places=12)
            self.assertEqual(report['safety_margin'], .1)

    def test_sphere_clear_broad_phase_skips_unneeded_box_distance(self):
        geometry = [('test', [0., 0., 0.], .3)]
        capsules = [((1, 6), (10., 10., 10.), (10., 10., 11.), .05)]
        with patch('planning.algorithms.human_collision.segment_box_distances') as narrow:
            report = nearest_human_collision(
                geometry, capsules, .1, robot_boxes=[(np.zeros(3), np.eye(3), [.1] * 3)])
            self.assertEqual(report['status'], 'FAR')
            self.assertFalse(narrow.called)

    def test_all_pairs_keep_consistent_refined_clearances_for_retreat(self):
        geometries = [('g0', [0., 0., 0.], 1.), ('g1', [2., 0., 0.], .2)]
        boxes = [(np.zeros(3), np.eye(3), [.1, .2, .3]), None]
        capsules = [((1, 6), (.25, 0., 0.), (.25, 0., .1), .05),
                    ((2, 6), (10., 0., 0.), (10., 0., .1), .05)]
        report = nearest_human_collision(geometries, capsules, .1, True, boxes)
        self.assertEqual(len(report['pairs']), 4)
        self.assertAlmostEqual(report['pairs'][(0, 1, 6)], .1, places=12)
        self.assertAlmostEqual(report['pairs'][(0, 2, 6)], 9.85, places=12)
        self.assertAlmostEqual(report['pairs'][(1, 2, 6)], 7.75, places=12)


class MeshEnclosureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with contextlib.redirect_stdout(io.StringIO()):
            cls.planner = PathPlanner(
                ROOT / 'src/planning/data/workspace_graph.npz',
                ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml',
                ROOT / 'src/cra/config/robot.yaml')

    def setUp(self):
        self.checker = self.planner.collision_filter
        self.checker.enable_human_obstacles(True)

    def test_every_mesh_vertex_is_enclosed_in_every_transformed_box(self):
        checker = self.checker
        model, data = checker.model, checker.data
        rng = np.random.default_rng(29)
        joints = [np.zeros(5)] + [rng.uniform(checker.lower, checker.upper) for _ in range(8)]
        self.assertEqual(len(checker.robot_human_boxes), 5)
        for q in joints:
            checker._set_joint_configuration(q)
            for geom, (center, half) in checker.robot_human_boxes.items():
                mesh = int(model.geom_dataid[geom])
                first, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
                vertices = np.asarray(model.mesh_vert[first:first + count], dtype=float)
                self.assertTrue(np.all(np.abs(vertices - center) <= half))
                rotation = data.geom_xmat[geom].reshape(3, 3)
                world_vertices = vertices @ rotation.T + data.geom_xpos[geom]
                box_center = data.geom_xpos[geom] + rotation @ center
                recovered = (world_vertices - box_center) @ rotation
                self.assertTrue(np.all(np.abs(recovered) <= half + 1e-12))

    def test_proven_empty_space_witness_becomes_clear_without_changing_margin(self):
        checker = self.checker
        q = np.array([1.134299692043946, -.7427666646992823, 1.2493752976508696,
                      -.15844472423457295, .31801132845616553])
        point = (-.06805981376466272, -.45285402006787534, .02819515949642165)
        snapshot = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 6, point, point, .065),))
        checker.set_human_capsules(snapshot)
        with patch.object(checker, 'robot_human_boxes', {}):
            old = checker.human_collision_report(q)
            self.assertEqual(old['status'], 'NEAR')
            self.assertAlmostEqual(old['clearance'], .07, places=6)
        report = checker.human_collision_report(q)
        self.assertEqual(report['status'], 'FAR')
        self.assertGreater(report['clearance'], .22)
        self.assertTrue(checker.is_state_valid(q))
        self.assertEqual(checker.human_safety_margin, .10)

    def test_points_inside_checked_mesh_enclosures_still_block(self):
        checker = self.checker
        q = np.array([1.08, -.71, 1.24, -.16, .32])
        checker._set_joint_configuration(q)
        base = checker.data.xpos[checker.base_id].copy()
        rotation = checker.data.xmat[checker.base_id].reshape(3, 3).copy()
        for geom, (local_center, _) in checker.robot_human_boxes.items():
            geom_rotation = checker.data.geom_xmat[geom].reshape(3, 3)
            world = checker.data.geom_xpos[geom] + geom_rotation @ local_center
            point = FrameTransform.ROBOT_FROM_MUJOCO_BASE @ rotation.T @ (world - base)
            checker.set_human_capsules(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(1, 6, tuple(point), tuple(point), .065),)))
            report = checker.human_collision_report(q)
            self.assertEqual(report['status'], 'NEAR')
            self.assertLess(report['clearance'], 0.)
            self.assertFalse(checker.is_state_valid(q))

    def test_capsules_touching_actual_mesh_vertices_cannot_be_classified_clear(self):
        checker = self.checker
        q = np.array([.7, -1.1, 1.8, -.4, .2])
        checker._set_joint_configuration(q)
        model, data = checker.model, checker.data
        base = data.xpos[checker.base_id].copy()
        base_rotation = data.xmat[checker.base_id].reshape(3, 3).copy()
        for geom in sorted(checker.robot_geoms):
            mesh = int(model.geom_dataid[geom])
            first, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
            vertices = model.mesh_vert[first:first + count]
            rotation = data.geom_xmat[geom].reshape(3, 3).copy()
            for vertex in vertices[np.linspace(0, count - 1, 20, dtype=int)]:
                world = data.geom_xpos[geom] + rotation @ vertex
                point = FrameTransform.ROBOT_FROM_MUJOCO_BASE @ base_rotation.T @ (world - base)
                checker.set_human_capsules(HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
                    HumanCapsule(1, 6, tuple(point), tuple(point), .005),)))
                report = checker.human_collision_report(q)
                self.assertEqual(report['status'], 'NEAR')
                self.assertLessEqual(report['clearance'], 0.)


if __name__ == '__main__':
    unittest.main()
