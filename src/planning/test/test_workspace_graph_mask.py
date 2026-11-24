"""XYZ display geometry, restoration, change-only publishing and planner isolation."""

import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from planning.planning_node import PlanningNode
from planning.visualization.workspace_graph_mask import WorkspaceGraphMask, segment_distances
from test_robot_visualization import load_visualizer, Node, ROOT, Marker


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.positions = np.array([[-1., 0., 0.], [1., 0., 0.], [0., 0., 0.],
                                   [-1., 1., 0.], [1., 1., 0.]])
        self.edges = np.array([[0, 1], [0, 3], [3, 4], [4, 1], [0, 2]])
        self.mask = WorkspaceGraphMask(self.positions, self.edges)
        self.capsule = ((0., 0., -.5), (0., 0., .5), .1)

    def test_capsule_hides_interior_node_and_crossing_edge_with_clear_endpoints(self):
        result = self.mask.update([self.capsule])
        np.testing.assert_array_equal(result['visible_node_mask'], [True, True, False, True, True])
        np.testing.assert_array_equal(result['visible_edge_mask'], [False, True, True, True, False])

    def test_obstacle_moves_away_restores_exact_original_graph(self):
        original = self.positions.copy(), self.edges.copy()
        self.mask.update([self.capsule])
        result = self.mask.update([((10., 0., 0.), (10., 0., 1.), .1)])
        self.assertTrue(result['visible_node_mask'].all())
        self.assertTrue(result['visible_edge_mask'].all())
        np.testing.assert_array_equal(original[0], self.mask.positions)
        np.testing.assert_array_equal(original[1], self.mask.edges)

    def test_parallel_tangent_degenerate_and_skew_segment_distances(self):
        starts = np.array([[-1., .1, 0.], [0., 0., 0.], [0., 1., 1.],
                           [-1., 0., 0.], [2., 0., 0.]])
        ends = np.array([[1., .1, 0.], [0., 0., 0.], [0., 1., 1.],
                         [1., 0., 0.], [3., 0., 0.]])
        distances = segment_distances(starts, ends, [-1., 0., 0.], [1., 0., 0.])
        np.testing.assert_allclose(distances, [.1, 0., np.sqrt(2.), 0., 1.], atol=1e-12)
        sphere = self.mask.update([((0., 0., 0.), (0., 0., 0.), .1)])
        self.assertFalse(sphere['visible_edge_mask'][0])

    def test_interior_crossing_and_nearly_parallel_crossing(self):
        starts = np.array([[0., -1., 0.], [-1., -1e-10, 0.]])
        ends = -starts
        np.testing.assert_allclose(segment_distances(
            starts, ends, [-1., 0., 0.], [1., 0., 0.]), [0., 0.], atol=1e-15)

    def test_capsule_boundary_is_hidden_and_two_capsules_form_union(self):
        mask = WorkspaceGraphMask([[0., .1, 0.], [0., .10001, 0.], [1., 1., 0.]], [])
        result = mask.update([self.capsule, ((1., 1., 0.), (1., 1., 0.), .01)])
        np.testing.assert_array_equal(result['visible_node_mask'], [False, True, False])

    def test_rotated_box_uses_its_local_axes(self):
        theta = np.pi / 4.
        rotation = np.array([[np.cos(theta), -np.sin(theta), 0.],
                             [np.sin(theta), np.cos(theta), 0.], [0., 0., 1.]])
        points = np.array([[0., .5, 0.], [.5, 0., 0.], [1., 0., 0.]]) @ rotation.T
        result = WorkspaceGraphMask(points, [[0, 1]]).update(
            boxes=[([0., 0., 0.], rotation, [.8, .1, .1])])
        np.testing.assert_array_equal(result['visible_node_mask'], [True, False, True])
        self.assertFalse(result['visible_edge_mask'][0])

    def test_box_crossing_is_hidden_with_both_endpoints_outside(self):
        result = self.mask.update(boxes=[([0., 0., 0.], np.eye(3), [.1, .1, .1])])
        self.assertTrue(result['visible_node_mask'][:2].all())
        self.assertFalse(result['visible_edge_mask'][0])

    def test_identical_geometry_does_no_distance_work(self):
        self.mask.update([self.capsule])
        with patch('planning.visualization.workspace_graph_mask.segment_distances',
                   side_effect=AssertionError('Unexpected geometry work')):
            self.assertIsNone(self.mask.update([self.capsule]))
        self.assertEqual(self.mask.calculations, 1)

    def test_changed_geometry_without_visibility_change_does_not_publish(self):
        self.mask.update([((10., 0., 0.), (10., 0., 1.), .1)])
        self.assertIsNone(self.mask.update([((11., 0., 0.), (11., 0., 1.), .1)]))
        self.assertEqual(self.mask.calculations, 2)

    def test_unknown_restores_graph_as_unverified_not_collision_free(self):
        self.mask.update([self.capsule])
        result = self.mask.update(status='UNKNOWN')
        self.assertFalse(result['known'])
        self.assertTrue(result['visible_node_mask'].all())
        self.assertTrue(result['visible_edge_mask'].all())

    def test_unknown_human_still_masks_available_boxes(self):
        result = self.mask.update(status='UNKNOWN', boxes=[
            ([0., 0., 0.], np.eye(3), [.1, .1, .1])])
        self.assertFalse(result['known'])
        self.assertFalse(result['visible_node_mask'][2])
        self.assertFalse(result['visible_edge_mask'][0])

    def test_empty_graph_and_no_obstacles(self):
        for points in (np.empty((0, 3)), self.positions):
            result = WorkspaceGraphMask(points, []).update(status='EMPTY')
            self.assertTrue(result['visible_node_mask'].all())
            self.assertTrue(result['known'])

    def test_invalid_geometry_is_explicit(self):
        for capsule in (([0., 0.], [0., 0., 0.], .1),
                        ([0., 0., 0.], [0., 0., 0.], -1.)):
            with self.assertRaises(ValueError):
                self.mask.update([capsule])
        with self.assertRaises(ValueError):
            self.mask.update(boxes=[([0., 0., 0.], np.zeros((3, 3)), [1., 1., 1.])])
        with self.assertRaises(ValueError):
            WorkspaceGraphMask(self.positions, [[0, 20]])


class DisplayIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.transport = Node()
        self.visualizer = object.__new__(load_visualizer())
        self.visualizer.node = self.transport
        self.visualizer.frame_id = 'base_link'
        self.visualizer.gng_nodes = np.array([[-1., 0., 0.], [1., 0., 0.], [0., 0., 0.]])
        self.visualizer.gng_edges = np.array([[0, 1], [0, 2]])
        self.messages, self.status, self.errors = [], [], []
        self.interested = True
        self.visualizer.workspace_mask_pub = SimpleNamespace(
            publish=self.messages.append, get_subscription_count=lambda: int(self.interested))
        self.checker = SimpleNamespace(
            environment_revision=1, base_id=0, human_data_ready=lambda: True,
            human_capsules=(((0, 0), (0., 0., -.5), (0., 0., .5), .1),),
            model=SimpleNamespace(geom_contype=np.array([0]), geom_conaffinity=np.array([0]),
                                  geom_type=np.array([6]), geom_size=np.array([[.1, .1, .1]])),
            data=SimpleNamespace(xpos=np.zeros((1, 3)), xmat=np.eye(3).reshape(1, 9),
                                 geom_xpos=np.zeros((1, 3)), geom_xmat=np.eye(3).reshape(1, 9)),
            obstacle_geoms={'box': {'geom_ids': [0]}})
        self.node = SimpleNamespace(
            workspace_graph_mask=WorkspaceGraphMask(self.visualizer.gng_nodes,
                                                    self.visualizer.gng_edges),
            workspace_mask_scene_key=None, visualizer=self.visualizer,
            workspace_mask_status_pub=SimpleNamespace(publish=self.status.append),
            planner=SimpleNamespace(collision_filter=self.checker),
            manager=SimpleNamespace(obstacle_data_ready=lambda: True,
                                    human_avoidance_enabled=True, human_status='OBSERVED',
                                    human_empty_until=None, pending_action=None),
            get_logger=lambda: SimpleNamespace(error=self.errors.append))
        self.node.update_workspace_graph_mask = lambda: PlanningNode.update_workspace_graph_mask(
            self.node)

    def update(self):
        PlanningNode.update_graph_visualization(self.node)

    def test_marker_visibility_and_persistent_lifetime(self):
        self.update()
        nodes, edges, text = self.messages[-1].markers
        self.assertEqual(len(nodes.points), 2)
        self.assertFalse(edges.points)
        self.assertEqual(nodes.header.frame_id, 'base_link')
        self.assertEqual(nodes.lifetime.sec, 0)
        self.assertEqual(nodes.lifetime.nanosec, 0)
        self.assertIn('Display only', text.text)

    def test_no_subscriber_performs_no_calculation_or_publication(self):
        self.interested = False
        self.update()
        self.assertFalse(self.messages)
        self.assertEqual(self.node.workspace_graph_mask.calculations, 0)

    def test_fresh_identical_updates_do_not_repeat_markers(self):
        self.update()
        for _ in range(20):
            self.checker.environment_revision += 1
            self.update()
        self.assertEqual(len(self.messages), 1)
        self.assertEqual(self.node.workspace_graph_mask.calculations, 1)

    def test_human_geometry_changes_even_without_checker_revision_change(self):
        self.update()
        self.checker.human_capsules = (((0, 0), (10., 0., 0.), (10., 0., 1.), .1),)
        self.update()
        self.assertEqual(len(self.messages[-1].markers[0].points), 3)
        self.assertEqual(self.node.workspace_graph_mask.calculations, 2)

    def test_expired_or_stale_input_keeps_graph_visible_without_motion(self):
        self.update()
        self.checker.human_data_ready = lambda: False
        self.update()
        nodes, edges, text = self.messages[-1].markers
        self.assertEqual(len(nodes.points), 3)
        self.assertEqual((nodes.color.r, nodes.color.g, nodes.color.a), (0., 1., .85))
        self.assertEqual(edges.color.a, .35)
        self.assertIn('UNKNOWN', text.text)
        self.node.manager.obstacle_data_ready = lambda: False
        self.update()
        self.assertIn('OBSTACLES_STALE', self.messages[-1].markers[-1].text)
        self.assertIsNone(self.node.manager.pending_action)

    def test_camera_absent_keeps_target_box_mask_and_original_colors(self):
        self.checker.human_data_ready = lambda: False
        self.checker.model.geom_contype[0] = 1
        self.update()
        nodes, edges, text = self.messages[-1].markers
        self.assertEqual(len(nodes.points), 2)
        self.assertFalse(edges.points)
        self.assertEqual((nodes.color.g, nodes.color.a, edges.color.a), (1., .85, .35))
        self.assertIn('UNKNOWN', text.text)

    def test_stale_simulation_poses_do_not_disable_fresh_human_mask(self):
        self.node.manager.obstacle_data_ready = lambda: False
        self.update()
        nodes, edges, text = self.messages[-1].markers
        self.assertEqual(len(nodes.points), 2)
        self.assertIn('OBSTACLES_STALE', text.text)

    def test_empty_and_disabled_human_use_only_existing_boxes(self):
        for status in ('EMPTY', 'HUMAN_DISABLED'):
            self.node.manager.human_status = status
            self.node.manager.human_avoidance_enabled = status != 'HUMAN_DISABLED'
            self.node.manager.human_empty_until = time.monotonic() + 1.
            self.checker.model.geom_contype[0] = 1
            self.update()
            self.assertEqual(len(self.messages[-1].markers[0].points), 2)
            self.assertIn(status, self.messages[-1].markers[-1].text)

    def test_display_never_runs_fk_or_collision_queries_or_changes_model(self):
        before = self.checker.data.xpos.copy(), self.checker.model.geom_size.copy()
        with patch('planning.planning_node.mujoco.mj_forward',
                   side_effect=AssertionError('Unexpected FK')):
            self.update()
        np.testing.assert_array_equal(before[0], self.checker.data.xpos)
        np.testing.assert_array_equal(before[1], self.checker.model.geom_size)
        self.assertIsNone(self.node.manager.pending_action)
        self.assertFalse(self.errors)

    def test_renderer_failure_disables_only_mask(self):
        with patch.object(self.visualizer, 'publish_workspace_graph_mask',
                          side_effect=ValueError('bad marker')):
            self.update()
        self.assertTrue(self.errors)
        self.assertIsNone(self.node.workspace_graph_mask)
        self.assertIsNone(self.node.manager.pending_action)
        self.assertEqual(self.messages[-1].markers[0].action, Marker.DELETE)

    def test_original_gng_and_mask_use_same_stored_coordinates(self):
        self.visualizer.gng_nodes, self.visualizer.gng_edges = self.visualizer.load_gng(
            ROOT / 'src/planning/data/workspace_graph.npz')
        self.node.workspace_graph_mask = WorkspaceGraphMask(
            self.visualizer.gng_nodes, self.visualizer.gng_edges)
        self.node.manager.human_avoidance_enabled = False
        self.update()
        array = self.messages[-1]
        self.assertEqual(len(array.markers[0].points), 1000)
        self.assertEqual(len(array.markers[1].points), 4453 * 2)
        original_pub = SimpleNamespace(publish=lambda _: None)
        self.visualizer.gng_nodes_pub = self.visualizer.gng_edges_pub = original_pub
        original = self.visualizer.publish_gng_nodes()
        np.testing.assert_array_equal(
            [(p.x, p.y, p.z) for p in original.points],
            [(p.x, p.y, p.z) for p in array.markers[0].points])

    def test_rviz_replaces_heavy_display_and_avoids_default_overlay(self):
        config = yaml.safe_load((ROOT / 'src/cra/config/cra.rviz').read_text())
        displays = {d['Name']: d for d in config['Visualization Manager']['Displays']}
        self.assertNotIn('Joint graph - filtered', displays)
        self.assertFalse(displays['GNG - original']['Enabled'])
        masked = displays['Workspace graph - obstacle mask']
        self.assertTrue(masked['Enabled'])
        self.assertEqual(masked['Topic']['Value'], '/planning/workspace_graph_mask')


if __name__ == '__main__':
    unittest.main()
