#!/usr/bin/env python3

import argparse
import heapq
from pathlib import Path

import numpy as np


class AStarPlanner:
    def __init__(
        self,
        joint_q,
        joint_edges,
        joint_costs,
        collision_filter,
        max_joint_delta=0.6,
        connection_count=32,
        local_connection_count=12,
    ):
        self.q = np.asarray(joint_q, dtype=float)
        edges = np.asarray(joint_edges, dtype=np.int64).reshape(-1, 2)
        costs = np.asarray(joint_costs, dtype=float)

        if (
            self.q.ndim != 2
            or self.q.shape[1] != 5
            or len(self.q) == 0
            or not np.isfinite(self.q).all()
        ):
            raise ValueError("Invalid joint graph configurations")

        if len(edges) and (
            edges.min() < 0 or edges.max() >= len(self.q)
        ):
            raise ValueError("Invalid joint graph edges")

        if (
            costs.shape != (len(edges),)
            or not np.isfinite(costs).all()
            or np.any(costs < 0.0)
        ):
            raise ValueError("Invalid joint graph costs")

        if not np.isfinite(max_joint_delta) or max_joint_delta <= 0:
            raise ValueError("max_joint_delta must be positive")

        if connection_count < 1 or local_connection_count < 1:
            raise ValueError("Connection counts must be positive")

        expected_costs = np.linalg.norm(
            self.q[edges[:, 0]] - self.q[edges[:, 1]],
            axis=1,
        )

        if not np.allclose(
            costs, expected_costs, rtol=1e-4, atol=1e-6
        ):
            raise ValueError(
                "Joint edge costs do not match joint distances"
            )

        self.checker = collision_filter
        self.max_joint_delta = float(max_joint_delta)
        self.connection_count = int(connection_count)
        self.local_connection_count = int(local_connection_count)

        self.adjacency = [[] for _ in range(len(self.q))]
        self.graph_neighbors = [set() for _ in range(len(self.q))]

        for (a, b), cost in zip(edges, costs):
            a, b = int(a), int(b)

            if a == b:
                continue

            self.adjacency[a].append((b, float(cost)))
            self.adjacency[b].append((a, float(cost)))

            self.graph_neighbors[a].add(b)
            self.graph_neighbors[b].add(a)

        self.revision = None
        self.state_cache = {}
        self.edge_cache = {}

        self.last_used_local_connections = False

    def _check_revision(self):
        if self.checker.environment_revision != self.revision:
            raise RuntimeError(
                "Obstacle changed during path planning"
            )

    def _state_valid(self, index, configuration):
        self._check_revision()

        if index not in self.state_cache:
            self.state_cache[index] = bool(
                self.checker.is_state_valid(configuration)
            )

        self._check_revision()
        return self.state_cache[index]

    def _edge_valid(self, a, b, qa, qb):
        self._check_revision()

        key = (min(a, b), max(a, b))

        if key not in self.edge_cache:
            self.edge_cache[key] = bool(
                self.checker.is_edge_valid(qa, qb)
            )

        self._check_revision()
        return self.edge_cache[key]

    def _nearby(self, configuration):
        delta = np.abs(self.q - configuration)

        candidates = np.flatnonzero(
            np.max(delta, axis=1) <= self.max_joint_delta
        )

        if len(candidates) == 0:
            return []

        distances = np.linalg.norm(
            self.q[candidates] - configuration,
            axis=1,
        )

        order = np.argsort(
            distances, kind="stable"
        )[:self.connection_count]

        return [
            (int(candidates[i]), float(distances[i]))
            for i in order
        ]

    def _local_neighbors(self, index, frontier_mask):
        delta = np.abs(self.q - self.q[index])

        candidates = np.flatnonzero(
            (np.max(delta, axis=1) <= self.max_joint_delta)
            & ~frontier_mask
        )

        candidates = [
            int(i)
            for i in candidates
            if (
                i != index
                and i not in self.graph_neighbors[index]
            )
        ]

        if not candidates:
            return []

        candidates = np.asarray(candidates, dtype=np.int64)

        distances = np.linalg.norm(
            self.q[candidates] - self.q[index],
            axis=1,
        )

        order = np.argsort(
            distances, kind="stable"
        )[:self.local_connection_count]

        return [
            (int(candidates[i]), float(distances[i]))
            for i in order
        ]

    def _search(
        self,
        start,
        goal,
        extra,
        use_local=False,
        frontier=None,
    ):
        n = len(self.q)
        start_id = n
        goal_id = n + 1

        def configuration(index):
            if index == start_id:
                return start
            if index == goal_id:
                return goal
            return self.q[index]

        g_score = {start_id: 0.0}
        predecessor = {}

        queue = [
            (
                float(np.linalg.norm(start - goal)),
                0.0,
                start_id,
            )
        ]

        expanded = set()
        frontier_mask = None

        if use_local and frontier is not None:
            frontier_mask = np.zeros(n, dtype=bool)
            frontier_mask[
                [i for i in frontier if i < n]
            ] = True

        while queue:
            self._check_revision()

            _, cost_so_far, current = heapq.heappop(queue)

            if (
                cost_so_far
                > g_score.get(current, float("inf")) + 1e-10
            ):
                continue

            if current in expanded:
                continue

            expanded.add(current)

            if current == goal_id:
                indices = [current]

                while indices[-1] != start_id:
                    indices.append(
                        predecessor[indices[-1]]
                    )

                indices.reverse()

                return np.vstack([
                    configuration(i) for i in indices
                ]), expanded

            neighbors = list(extra.get(current, ()))

            if current < n:
                neighbors.extend(self.adjacency[current])

                if (
                    use_local
                    and frontier_mask is not None
                    and current in frontier
                ):
                    neighbors.extend(
                        self._local_neighbors(
                            current,
                            frontier_mask,
                        )
                    )

            current_q = configuration(current)

            for neighbor, edge_cost in neighbors:
                if neighbor in expanded:
                    continue

                neighbor_q = configuration(neighbor)

                if not self._state_valid(
                    neighbor, neighbor_q
                ):
                    continue

                if not self._edge_valid(
                    current,
                    neighbor,
                    current_q,
                    neighbor_q,
                ):
                    continue

                proposed = cost_so_far + edge_cost

                if proposed + 1e-10 >= g_score.get(
                    neighbor, float("inf")
                ):
                    continue

                g_score[neighbor] = proposed
                predecessor[neighbor] = current

                heuristic = float(
                    np.linalg.norm(neighbor_q - goal)
                )

                heapq.heappush(
                    queue,
                    (
                        proposed + heuristic,
                        proposed,
                        neighbor,
                    ),
                )

        return None, expanded

    def plan(self, start_q, goal_q):
        start = np.asarray(start_q, dtype=float)
        goal = np.asarray(goal_q, dtype=float)

        if (
            start.shape != (5,)
            or goal.shape != (5,)
            or not np.isfinite(start).all()
            or not np.isfinite(goal).all()
        ):
            raise ValueError(
                "Start and goal must be finite 5-DOF configurations"
            )

        revision = self.checker.environment_revision

        if self.revision != revision:
            self.revision = revision
            self.state_cache.clear()
            self.edge_cache.clear()

        n = len(self.q)
        start_id = n
        goal_id = n + 1

        # Temporary configurations change for every planning request.
        self.state_cache.pop(start_id, None)
        self.state_cache.pop(goal_id, None)

        self.edge_cache = {
            key: valid
            for key, valid in self.edge_cache.items()
            if key[1] < n
        }

        self.last_used_local_connections = False

        if not self._state_valid(start_id, start):
            print("[A*] NO_PATH: invalid start", flush=True)
            return None

        if not self._state_valid(goal_id, goal):
            print("[A*] NO_PATH: invalid goal", flush=True)
            return None

        if np.array_equal(start, goal):
            return start[None, :].copy()

        extra = {
            start_id: [],
            goal_id: [],
        }

        start_neighbors = self._nearby(start)
        goal_neighbors = self._nearby(goal)

        # Diagnose the closest start connection.
        start_delta = np.abs(self.q - start)
        start_distances = np.linalg.norm(
            start_delta, axis=1
        )

        nearest_start = int(np.argmin(start_distances))
        nearest_delta = start_delta[nearest_start]

        print(
            f"[A*] Start candidates: {len(start_neighbors)}, "
            f"Goal candidates: {len(goal_neighbors)}",
            flush=True,
        )

        print(
            f"[A*] Nearest start node: {nearest_start}, "
            f"distance={start_distances[nearest_start]:.3f} rad, "
            f"max_delta={nearest_delta.max():.3f} rad, "
            f"limit={self.max_joint_delta:.3f} rad",
            flush=True,
        )

        print(
            f"[A*] Start joints: "
            f"{np.round(start, 3).tolist()}",
            flush=True,
        )

        print(
            f"[A*] Nearest graph joints: "
            f"{np.round(self.q[nearest_start], 3).tolist()}",
            flush=True,
        )

        for index, cost in start_neighbors:
            extra[start_id].append((index, cost))
            extra.setdefault(index, []).append(
                (start_id, cost)
            )

        for index, cost in goal_neighbors:
            extra[goal_id].append((index, cost))
            extra.setdefault(index, []).append(
                (goal_id, cost)
            )

        if (
            np.max(np.abs(start - goal))
            <= self.max_joint_delta
        ):
            distance = float(
                np.linalg.norm(start - goal)
            )

            extra[start_id].append(
                (goal_id, distance)
            )
            extra[goal_id].append(
                (start_id, distance)
            )

        result, reached = self._search(
            start,
            goal,
            extra,
        )

        print(
            f"[A*] First search expanded: {len(reached)} states",
            flush=True,
        )

        if result is not None:
            print(
                f"[A*] PATH_FOUND: {len(result)} waypoints",
                flush=True,
            )
            return result

        result, reached_local = self._search(
            start,
            goal,
            extra,
            use_local=True,
            frontier=reached,
        )

        self.last_used_local_connections = (
            result is not None
        )

        print(
            f"[A*] Local search expanded: "
            f"{len(reached_local)} states",
            flush=True,
        )

        if result is None:
            print(
                "[A*] NO_PATH: no collision-free connection",
                flush=True,
            )
        else:
            print(
                "[A*] PATH_FOUND: "
                f"{len(result)} waypoints "
                "(local connections used)",
                flush=True,
            )

        return result


def main():
    import yaml

    from planning.algorithms.collision_filter import CollisionFilter

    parser = argparse.ArgumentParser(
        description="Offline CRA joint-space A* test"
    )

    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--start-index", type=int, required=True)
    parser.add_argument("--goal-index", type=int, required=True)

    args = parser.parse_args()

    with args.config.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)

    with np.load(args.graph, allow_pickle=False) as graph:
        joints = graph["joint_q"].copy()
        edges = graph["joint_edges"].copy()
        costs = graph["joint_costs"].copy()

    if not (
        0 <= args.start_index < len(joints)
        and 0 <= args.goal_index < len(joints)
    ):
        parser.error("Invalid start or goal index")

    checker = CollisionFilter(
        model_path=args.model,
        joint_names=list(config["robot"]["joints"]),
        safety_margin=float(
            config["planning"]["obstacle_safety_margin"]
        ),
    )

    planner = AStarPlanner(
        joint_q=joints,
        joint_edges=edges,
        joint_costs=costs,
        collision_filter=checker,
        max_joint_delta=float(
            config["planning"]["joint_graph"]["max_joint_delta"]
        ),
    )

    print("CRA Joint-Space A* Test")
    print(f"Joint nodes: {len(joints)}")
    print(f"Joint edges: {len(edges)}")

    path = planner.plan(
        joints[args.start_index],
        joints[args.goal_index],
    )

    if path is None:
        print("No collision-free path found")
        return

    distance = float(
        np.linalg.norm(
            np.diff(path, axis=0), axis=1
        ).sum()
    )

    print("\nPath found")
    print(f"Waypoints: {len(path)}")
    print(f"Joint distance: {distance:.4f} rad")
    print(
        f"Local connections used: "
        f"{planner.last_used_local_connections}"
    )


if __name__ == "__main__":
    main()