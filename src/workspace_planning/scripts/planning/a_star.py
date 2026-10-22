#!/usr/bin/env python3

import heapq
import numpy as np


class AStarPlanner:

    def __init__(self, positions, edges, q=None):

        # Memuat graf GNG
        self.positions = np.asarray(
            positions, dtype=float
        )

        self.edges = np.asarray(
            edges, dtype=int
        ).reshape(-1, 2)

        self.q = (
            None if q is None
            else np.asarray(q, dtype=float)
        )

        if (
            self.positions.ndim != 2
            or self.positions.shape[1] != 3
        ):
            raise ValueError(
                "positions harus berbentuk (N,3)"
            )

        if self.q is not None:
            if len(self.q) != len(self.positions):
                raise ValueError(
                    "Jumlah q dan positions berbeda"
                )

        self.valid_nodes = np.ones(
            len(self.positions), dtype=bool
        )

        self.valid_edges = np.ones(
            len(self.edges), dtype=bool
        )

        # Edge yang ditolak berdasarkan collision
        self.rejected_edges = set()

        self.adjacency = []
        self._build_adjacency()

    # Memperbarui validitas node dan edge
    def set_graph_validity(
        self,
        valid_nodes,
        valid_edges
    ):

        self.valid_nodes = np.asarray(
            valid_nodes, dtype=bool
        ).copy()

        self.valid_edges = np.asarray(
            valid_edges, dtype=bool
        ).copy()

        if len(self.valid_nodes) != len(self.positions):
            raise ValueError("Jumlah valid_nodes salah")

        if len(self.valid_edges) != len(self.edges):
            raise ValueError("Jumlah valid_edges salah")

        self._build_adjacency()

    # Menolak satu edge
    def block_edge(self, a, b):

        edge = tuple(sorted((
            int(a), int(b)
        )))

        self.rejected_edges.add(edge)
        self._build_adjacency()

    # Menghapus edge yang sebelumnya ditolak
    def reset_rejected_edges(self):

        self.rejected_edges.clear()
        self._build_adjacency()

    # Membentuk adjacency
    def _build_adjacency(self):

        self.adjacency = [
            [] for _ in range(len(self.positions))
        ]

        for index, (a, b) in enumerate(self.edges):

            a = int(a)
            b = int(b)

            if not self.valid_edges[index]:
                continue

            if not (
                self.valid_nodes[a]
                and self.valid_nodes[b]
            ):
                continue

            edge = tuple(sorted((a, b)))

            if edge in self.rejected_edges:
                continue

            cost = float(
                np.linalg.norm(
                    self.positions[a]
                    - self.positions[b]
                )
            )

            self.adjacency[a].append((b, cost))
            self.adjacency[b].append((a, cost))

    # Menghitung heuristik A*
    def _heuristic(self, node, goal):

        return float(
            np.linalg.norm(
                self.positions[node]
                - self.positions[goal]
            )
        )

    # Menjalankan pencarian A*
    def _search(self, start, goal):

        queue = [
            (self._heuristic(start, goal), start)
        ]

        distance = {start: 0.0}
        previous = {}
        visited = set()

        while queue:

            _, current = heapq.heappop(queue)

            if current in visited:
                continue

            if current == goal:

                result = [current]

                while current in previous:
                    current = previous[current]
                    result.append(current)

                result.reverse()
                return result

            visited.add(current)

            for neighbor, cost in self.adjacency[current]:

                if neighbor in visited:
                    continue

                candidate = distance[current] + cost

                if candidate < distance.get(
                    neighbor, float("inf")
                ):

                    distance[neighbor] = candidate
                    previous[neighbor] = current

                    heapq.heappush(
                        queue,
                        (
                            candidate
                            + self._heuristic(
                                neighbor, goal
                            ),
                            neighbor
                        )
                    )

        return None

    # Mengidentifikasi komponen graf
    def _connected_components(self):

        visited = set()
        components = []

        for node in np.flatnonzero(self.valid_nodes):

            node = int(node)

            if node in visited:
                continue

            stack = [node]
            visited.add(node)
            component = []

            while stack:

                current = stack.pop()
                component.append(current)

                for neighbor, _ in self.adjacency[current]:

                    if neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)

            components.append(
                np.asarray(component, dtype=int)
            )

        return components

    # Menghitung panjang lintasan
    def _path_length(self, node_path):

        if len(node_path) < 2:
            return 0.0

        points = self.positions[node_path]

        return float(
            np.sum(
                np.linalg.norm(
                    np.diff(points, axis=0),
                    axis=1
                )
            )
        )

    # Merencanakan jalur menggunakan A*
    def plan(
        self,
        start_position,
        goal_position,
        start_connector_valid=None,
        goal_connector_valid=None
    ):

        start_position = np.asarray(
            start_position, dtype=float
        )

        goal_position = np.asarray(
            goal_position, dtype=float
        )

        n = len(self.positions)

        if start_connector_valid is None:
            start_connector_valid = np.ones(
                n, dtype=bool
            )

        if goal_connector_valid is None:
            goal_connector_valid = np.ones(
                n, dtype=bool
            )

        start_connector_valid = np.asarray(
            start_connector_valid, dtype=bool
        )

        goal_connector_valid = np.asarray(
            goal_connector_valid, dtype=bool
        )

        if (
            start_connector_valid.shape != (n,)
            or goal_connector_valid.shape != (n,)
        ):
            raise ValueError(
                "Ukuran connector mask salah"
            )

        best = None

        for component in self._connected_components():

            starts = component[
                start_connector_valid[component]
            ]

            goals = component[
                goal_connector_valid[component]
            ]

            if len(starts) == 0 or len(goals) == 0:
                continue

            start = int(starts[np.argmin(
                np.linalg.norm(
                    self.positions[starts]
                    - start_position,
                    axis=1
                )
            )])

            goal = int(goals[np.argmin(
                np.linalg.norm(
                    self.positions[goals]
                    - goal_position,
                    axis=1
                )
            )])

            node_path = self._search(
                start, goal
            )

            if node_path is None:
                continue

            path_length = self._path_length(
                node_path
            )

            start_distance = float(
                np.linalg.norm(
                    start_position
                    - self.positions[start]
                )
            )

            goal_distance = float(
                np.linalg.norm(
                    goal_position
                    - self.positions[goal]
                )
            )

            total = (
                start_distance
                + path_length
                + goal_distance
            )

            if (
                best is None
                or total < best["total_length"]
            ):

                best = {
                    "start_node": start,
                    "goal_node": goal,
                    "node_path": np.asarray(
                        node_path, dtype=int
                    ),
                    "path_length": path_length,
                    "start_distance": start_distance,
                    "goal_distance": goal_distance,
                    "total_length": total
                }

        if best is None:
            raise RuntimeError(
                "A* tidak menemukan jalur valid"
            )

        ids = best["node_path"]

        result = dict(best)
        result["positions"] = self.positions[ids]

        if self.q is not None:
            result["q"] = self.q[ids]

        return result