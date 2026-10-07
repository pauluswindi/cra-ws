#!/usr/bin/env python3

import heapq
import numpy as np


class AStarPlanner:
    def __init__(self, positions, edges, q=None):
        self.positions = np.asarray(positions, dtype=np.float64)
        self.edges = np.asarray(edges, dtype=np.int64)

        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError("Positions must have shape (N, 3).")

        if self.edges.ndim != 2 or self.edges.shape[1] != 2:
            raise ValueError("Edges must have shape (M, 2).")

        self.q = None if q is None else np.asarray(q, dtype=np.float64)

        if self.q is not None and self.q.shape[0] != self.positions.shape[0]:
            raise ValueError("q and positions must have the same number of nodes.")

        self.adjacency = self._build_adjacency()

    def _build_adjacency(self):
        adjacency = [[] for _ in range(len(self.positions))]

        for node_a, node_b in self.edges:
            node_a = int(node_a)
            node_b = int(node_b)

            if node_a < 0 or node_a >= len(self.positions):
                continue

            if node_b < 0 or node_b >= len(self.positions):
                continue

            cost = np.linalg.norm(
                self.positions[node_a] - self.positions[node_b]
            )

            adjacency[node_a].append((node_b, cost))
            adjacency[node_b].append((node_a, cost))

        return adjacency

    def _nearest_node(self, position):
        position = np.asarray(position, dtype=np.float64)

        distances = np.linalg.norm(
            self.positions - position,
            axis=1,
        )

        return int(np.argmin(distances))

    def _heuristic(self, node, goal_node):
        return np.linalg.norm(
            self.positions[node] - self.positions[goal_node]
        )

    def _reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def _search(self, start_node, goal_node):
        open_set = []

        heapq.heappush(
            open_set,
            (
                self._heuristic(start_node, goal_node),
                start_node,
            ),
        )

        came_from = {}

        g_score = {
            start_node: 0.0,
        }

        f_score = {
            start_node: self._heuristic(start_node, goal_node),
        }

        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal_node:
                return self._reconstruct_path(
                    came_from,
                    current,
                )

            closed_set.add(current)

            for neighbor, edge_cost in self.adjacency[current]:
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + edge_cost

                if tentative_g < g_score.get(neighbor, np.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score[neighbor] = (
                        tentative_g
                        + self._heuristic(neighbor, goal_node)
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score[neighbor],
                            neighbor,
                        ),
                    )

        return None

    def plan(self, start_position, goal_position):
        start_position = np.asarray(
            start_position,
            dtype=np.float64,
        )

        goal_position = np.asarray(
            goal_position,
            dtype=np.float64,
        )

        if start_position.shape != (3,):
            raise ValueError("Start position must have shape (3,).")

        if goal_position.shape != (3,):
            raise ValueError("Goal position must have shape (3,).")

        start_node = self._nearest_node(start_position)
        goal_node = self._nearest_node(goal_position)

        node_path = self._search(
            start_node,
            goal_node,
        )

        if node_path is None:
            raise RuntimeError(
                "No path found between start and goal."
            )

        result = {
            "start_node": start_node,
            "goal_node": goal_node,
            "node_path": np.asarray(node_path, dtype=np.int64),
            "positions": self.positions[node_path],
            "start_distance": np.linalg.norm(
                self.positions[start_node] - start_position
            ),
            "goal_distance": np.linalg.norm(
                self.positions[goal_node] - goal_position
            ),
        }

        if self.q is not None:
            result["q"] = self.q[node_path]

        result["path_length"] = self._calculate_path_length(node_path)

        return result

    def _calculate_path_length(self, node_path):
        if len(node_path) < 2:
            return 0.0

        positions = self.positions[node_path]

        return float(
            np.sum(
                np.linalg.norm(
                    positions[1:] - positions[:-1],
                    axis=1,
                )
            )
        )