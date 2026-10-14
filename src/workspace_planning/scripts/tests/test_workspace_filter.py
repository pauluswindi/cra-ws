#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
PLANNING_DIR = TEST_DIR.parent / "planning"

sys.path.insert(0, str(PLANNING_DIR))

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "planning"))

from workspace_filter import WorkspaceFilter
from a_star import AStarPlanner


class WorkspaceFilterTests(unittest.TestCase):
    def setUp(self):
        self.positions = np.array(
            [
                [-0.3, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.3, 0.0, 0.0],
                [0.3, 0.3, 0.0],
            ],
            dtype=np.float64,
        )
        self.edges = np.array([[0, 1], [1, 2], [2, 3], [0, 2]], dtype=np.int64)
        self.filter = WorkspaceFilter(self.positions, self.edges, safety_margin=0.0)

    def test_box_blocks_node_and_crossing_edges(self):
        self.filter.set_obstacle("box", [0.0, 0.0, 0.0], [0.2, 0.2, 0.2])
        valid_nodes, valid_edges = self.filter.update()

        self.assertEqual(valid_nodes.tolist(), [True, False, True, True])
        self.assertEqual(valid_edges.tolist(), [False, False, True, False])

    def test_moving_obstacle_restores_previous_nodes(self):
        self.filter.set_obstacle("box", [0.0, 0.0, 0.0], [0.2, 0.2, 0.2])
        self.filter.update()
        self.filter.set_obstacle("box", [0.0, 0.8, 0.0], [0.2, 0.2, 0.2])
        valid_nodes, valid_edges = self.filter.update()

        self.assertTrue(valid_nodes.all())
        self.assertTrue(valid_edges.all())

    def test_target_bounds_and_obstacle_are_both_applied(self):
        self.filter.set_target_box([0.3, 0.3, 0.0], [0.08, 0.08, 0.08])
        self.filter.set_obstacle("sphere", [0.0, 0.0, 0.0], [0.12, 0.12, 0.12])
        valid_nodes, _ = self.filter.update()

        self.assertFalse(valid_nodes[1])
        self.assertFalse(valid_nodes[3])
        self.assertTrue(valid_nodes[2])

    def test_rotated_box_checks_segments(self):
        quarter_turn = [0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)]
        self.filter.set_obstacle("box", [0.0, 0.0, 0.0], [0.2, 0.4, 0.2], quarter_turn)
        self.assertTrue(self.filter.is_segment_blocked([-0.3, 0.0, 0.0], [0.3, 0.0, 0.0]))

    def test_astar_uses_filtered_graph_and_connector_masks(self):
        positions = np.array(
            [[0, 0, 0], [1, 0, 0], [2, 0, 0], [0, 1, 0], [1, 1, 0], [2, 1, 0]],
            dtype=np.float64,
        )
        edges = np.array([[0, 1], [1, 2], [3, 4], [4, 5]], dtype=np.int64)
        planner = AStarPlanner(positions, edges)
        planner.set_graph_validity(
            valid_nodes=[False, False, False, True, True, True],
            valid_edges=[False, False, True, True],
        )
        result = planner.plan(
            [0, 0, 0],
            [2, 0, 0],
            start_connector_valid=[False, False, False, True, True, True],
            goal_connector_valid=[False, False, False, True, True, True],
        )

        self.assertEqual(result["node_path"].tolist(), [3, 4, 5])


if __name__ == "__main__":
    unittest.main()
