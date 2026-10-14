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

        self.valid_nodes = np.ones(len(self.positions), dtype=bool)
        self.valid_edges = np.ones(len(self.edges), dtype=bool)
        self.adjacency = self._build_adjacency()

    def set_graph_validity(self, valid_nodes, valid_edges):
        """Update active nodes and edges without changing the original graph indices."""
        valid_nodes = np.asarray(valid_nodes, dtype=bool)
        valid_edges = np.asarray(valid_edges, dtype=bool)

        if valid_nodes.shape != (len(self.positions),):
            raise ValueError("valid_nodes must have one value per workspace node.")
        if valid_edges.shape != (len(self.edges),):
            raise ValueError("valid_edges must have one value per workspace edge.")

        self.valid_nodes = valid_nodes.copy()
        self.valid_edges = valid_edges.copy()
        self.adjacency = self._build_adjacency()

    def _build_adjacency(self):
        adjacency = [[] for _ in range(len(self.positions))]

        for edge_index, (node_a, node_b) in enumerate(self.edges):
            node_a = int(node_a)
            node_b = int(node_b)

            if not self.valid_edges[edge_index]:
                continue

            if node_a < 0 or node_a >= len(self.positions):
                continue

            if node_b < 0 or node_b >= len(self.positions):
                continue

            if not self.valid_nodes[node_a] or not self.valid_nodes[node_b]:
                continue

            cost = np.linalg.norm(
                self.positions[node_a] - self.positions[node_b]
            )

            adjacency[node_a].append((node_b, cost))
            adjacency[node_b].append((node_a, cost))

        return adjacency

    def _nearest_node(self, position):
        position = np.asarray(position, dtype=np.float64)
        valid_indices = np.flatnonzero(self.valid_nodes)

        if len(valid_indices) == 0:
            raise RuntimeError("No valid workspace nodes remain after filtering.")

        distances = np.linalg.norm(
            self.positions[valid_indices] - position,
            axis=1,
        )

        return int(valid_indices[int(np.argmin(distances))])

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

    def _connected_components(self):
        components = []
        visited = np.zeros(len(self.positions), dtype=bool)

        for start in range(len(self.positions)):
            if visited[start] or not self.valid_nodes[start]:
                continue

            component = []
            stack = [start]
            visited[start] = True

            while stack:
                node = stack.pop()
                component.append(node)
                for neighbor, _ in self.adjacency[node]:
                    if not visited[neighbor]:
                        visited[neighbor] = True
                        stack.append(neighbor)

            components.append(np.asarray(component, dtype=np.int64))

        return components

    def plan(
        self,
        start_position,
        goal_position,
        start_connector_valid=None,
        goal_connector_valid=None,
    ):
        start_position = np.asarray(start_position, dtype=np.float64)
        goal_position = np.asarray(goal_position, dtype=np.float64)

        if start_position.shape != (3,):
            raise ValueError("Start position must have shape (3,).")
        if goal_position.shape != (3,):
            raise ValueError("Goal position must have shape (3,).")
        if not np.all(np.isfinite(start_position)) or not np.all(np.isfinite(goal_position)):
            raise ValueError("Start and goal positions must contain finite values.")

        if start_connector_valid is None:
            start_connector_valid = np.ones(len(self.positions), dtype=bool)
        else:
            start_connector_valid = np.asarray(start_connector_valid, dtype=bool)
        if goal_connector_valid is None:
            goal_connector_valid = np.ones(len(self.positions), dtype=bool)
        else:
            goal_connector_valid = np.asarray(goal_connector_valid, dtype=bool)

        if start_connector_valid.shape != (len(self.positions),):
            raise ValueError("start_connector_valid must have one value per workspace node.")
        if goal_connector_valid.shape != (len(self.positions),):
            raise ValueError("goal_connector_valid must have one value per workspace node.")

        components = self._connected_components()
        if not components:
            raise RuntimeError("No valid workspace nodes remain after filtering.")

        best = None
        for component in components:
            start_candidates = component[start_connector_valid[component]]
            goal_candidates = component[goal_connector_valid[component]]
            if len(start_candidates) == 0 or len(goal_candidates) == 0:
                continue

            start_distances = np.linalg.norm(
                self.positions[start_candidates] - start_position,
                axis=1,
            )
            goal_distances = np.linalg.norm(
                self.positions[goal_candidates] - goal_position,
                axis=1,
            )
            start_node = int(start_candidates[int(np.argmin(start_distances))])
            goal_node = int(goal_candidates[int(np.argmin(goal_distances))])

            node_path = self._search(start_node, goal_node)
            if node_path is None:
                continue

            path_length = self._calculate_path_length(node_path)
            start_distance = float(
                np.linalg.norm(self.positions[start_node] - start_position)
            )
            goal_distance = float(
                np.linalg.norm(self.positions[goal_node] - goal_position)
            )
            total_length = start_distance + path_length + goal_distance

            if best is None or total_length < best["total_length"]:
                best = {
                    "start_node": start_node,
                    "goal_node": goal_node,
                    "node_path": np.asarray(node_path, dtype=np.int64),
                    "start_distance": start_distance,
                    "goal_distance": goal_distance,
                    "path_length": path_length,
                    "total_length": total_length,
                }

        if best is None:
            raise RuntimeError("No path found between start and goal in the filtered graph.")

        node_path = best["node_path"].tolist()
        result = {
            "start_node": best["start_node"],
            "goal_node": best["goal_node"],
            "node_path": best["node_path"],
            "positions": self.positions[node_path],
            "start_distance": best["start_distance"],
            "goal_distance": best["goal_distance"],
            "path_length": best["path_length"],
            "total_length": best["total_length"],
        }

        if self.q is not None:
            result["q"] = self.q[node_path]

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