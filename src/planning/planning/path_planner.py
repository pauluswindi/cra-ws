#!/usr/bin/env python3

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml

from planning.algorithms.a_star import AStarPlanner
from planning.algorithms.collision_filter import CollisionFilter


class PathPlanner:
    def __init__(self, graph_path, model_path, config_path):
        self.graph_path = Path(graph_path)
        self.model_path = Path(model_path)

        with Path(config_path).open(encoding="utf-8") as file:
            config = yaml.safe_load(file)

        self.joint_names = list(config["robot"]["joints"])

        with np.load(self.graph_path, allow_pickle=False) as graph:
            self.joint_q = np.asarray(
                graph["joint_q"], dtype=float
            )
            self.joint_edges = np.asarray(
                graph["joint_edges"], dtype=np.int64
            ).reshape(-1, 2)
            self.joint_costs = np.asarray(
                graph["joint_costs"], dtype=float
            )

        if len(self.joint_names) != 5:
            raise ValueError("Expected five robot joints")

        if (
            self.joint_q.ndim != 2
            or self.joint_q.shape[1] != 5
            or not np.isfinite(self.joint_q).all()
        ):
            raise ValueError("Invalid joint graph configurations")

        if len(self.joint_edges) and (
            self.joint_edges.min() < 0
            or self.joint_edges.max() >= len(self.joint_q)
        ):
            raise ValueError("Invalid joint graph edges")

        if (
            self.joint_costs.shape != (len(self.joint_edges),)
            or not np.isfinite(self.joint_costs).all()
            or np.any(self.joint_costs < 0.0)
        ):
            raise ValueError("Invalid joint graph costs")

        planning_config = config["planning"]
        joint_config = planning_config["joint_graph"]

        self.collision_filter = CollisionFilter(
            model_path=self.model_path,
            joint_names=self.joint_names,
            safety_margin=float(
                planning_config["obstacle_safety_margin"]
            ),
        )

        self.planner = AStarPlanner(
            joint_q=self.joint_q,
            joint_edges=self.joint_edges,
            joint_costs=self.joint_costs,
            collision_filter=self.collision_filter,
            max_joint_delta=float(
                joint_config["max_joint_delta"]
            ),
        )

        self.current_path = None
        self.goal_q = None
        self.path_revision = None

        self.valid_node_mask = None
        self.valid_edge_mask = None
        self.graph_filter_revision = None

    def invalidate_graph_filter(self):
        self.valid_node_mask = None
        self.valid_edge_mask = None
        self.graph_filter_revision = None

    def filter_joint_graph(self):
        checker = self.collision_filter
        revision = checker.environment_revision

        if (
            self.graph_filter_revision == revision
            and self.valid_node_mask is not None
            and self.valid_edge_mask is not None
        ):
            return (
                self.valid_node_mask.copy(),
                self.valid_edge_mask.copy(),
            )

        valid_nodes = np.array(
            [
                checker.is_state_valid(joints)
                for joints in self.joint_q
            ],
            dtype=bool,
        )

        valid_edges = np.zeros(
            len(self.joint_edges), dtype=bool
        )

        for index, (a, b) in enumerate(self.joint_edges):
            if valid_nodes[a] and valid_nodes[b]:
                valid_edges[index] = checker.is_edge_valid(
                    self.joint_q[a],
                    self.joint_q[b],
                )

        if checker.environment_revision != revision:
            self.invalidate_graph_filter()
            raise RuntimeError(
                "Environment changed during graph filtering"
            )

        self.valid_node_mask = valid_nodes
        self.valid_edge_mask = valid_edges
        self.graph_filter_revision = revision

        return valid_nodes.copy(), valid_edges.copy()

    def get_filtered_graph(self):
        valid_nodes, valid_edges = self.filter_joint_graph()

        return {
            "joint_q": self.joint_q.copy(),
            "joint_edges": self.joint_edges.copy(),
            "valid_node_mask": valid_nodes,
            "valid_edge_mask": valid_edges,
            "environment_revision": self.graph_filter_revision,
        }

    def get_filter_statistics(self):
        valid_nodes, valid_edges = self.filter_joint_graph()

        return {
            "total_nodes": len(valid_nodes),
            "valid_nodes": int(np.count_nonzero(valid_nodes)),
            "blocked_nodes": int(np.count_nonzero(~valid_nodes)),
            "total_edges": len(valid_edges),
            "valid_edges": int(np.count_nonzero(valid_edges)),
            "blocked_edges": int(np.count_nonzero(~valid_edges)),
            "environment_revision": self.graph_filter_revision,
        }

    def is_path_valid(self, path):
        path = np.asarray(path, dtype=float)

        if (
            path.ndim != 2
            or path.shape[1] != 5
            or len(path) == 0
            or not np.isfinite(path).all()
        ):
            return False

        checker = self.collision_filter
        revision = checker.environment_revision

        for joints in path:
            if not checker.is_state_valid(joints):
                return False

        for start, goal in zip(path[:-1], path[1:]):
            if not checker.is_edge_valid(start, goal):
                return False

        return checker.environment_revision == revision

    def plan(self, start_q, goal_q):
        start = np.asarray(start_q, dtype=float)
        goal = np.asarray(goal_q, dtype=float)

        if (
            start.shape != (5,)
            or goal.shape != (5,)
            or not np.isfinite(start).all()
            or not np.isfinite(goal).all()
        ):
            raise ValueError("Invalid start or goal configuration")

        self.current_path = None
        self.path_revision = None
        self.goal_q = goal.copy()

        checker = self.collision_filter
        revision = checker.environment_revision

        if not checker.is_state_valid(start):
            print(
                "[PathPlanner] NO_PATH: "
                "start configuration is invalid",
                flush=True,
            )
            return False

        if not checker.is_state_valid(goal):
            print(
                "[PathPlanner] NO_PATH: "
                "goal configuration is invalid",
                flush=True,
            )
            return False

        if checker.environment_revision != revision:
            print(
                "[PathPlanner] NO_PATH: environment changed",
                flush=True,
            )
            return False

        path = self.planner.plan(start, goal)

        if path is None:
            print(
                "[PathPlanner] NO_PATH: "
                "A* could not connect start to goal",
                flush=True,
            )
            return False

        if (
            checker.environment_revision != revision
            or not self.is_path_valid(path)
        ):
            print(
                "[PathPlanner] NO_PATH: "
                "final path validation failed",
                flush=True,
            )
            return False

        self.current_path = np.asarray(
            path, dtype=float
        ).copy()

        self.path_revision = revision

        print(
            "[PathPlanner] PATH_READY: "
            f"{len(self.current_path)} waypoints",
            flush=True,
        )

        return True

    def update_obstacle(
        self,
        name,
        position,
        quaternion=None,
        size=None,
    ):
        self.collision_filter.set_obstacle_pose(
            name=name,
            position=position,
            quaternion=quaternion,
            size=size,
        )

        self.path_revision = None
        self.invalidate_graph_filter()

    def remove_obstacle(self, name):
        self.collision_filter.remove_obstacle(name)

        self.path_revision = None
        self.invalidate_graph_filter()

    def replan_if_needed(self, current_q, remaining_path):
        if self.goal_q is None:
            return "NO_GOAL", None

        current = np.asarray(current_q, dtype=float)
        remaining = np.asarray(remaining_path, dtype=float)

        if (
            current.shape != (5,)
            or not np.isfinite(current).all()
        ):
            return "INVALID_CURRENT_STATE", None

        if remaining.size == 0:
            remaining = np.empty((0, 5), dtype=float)

        if (
            remaining.ndim != 2
            or remaining.shape[1] != 5
            or not np.isfinite(remaining).all()
        ):
            return "INVALID_REMAINING_PATH", None

        if not self.collision_filter.is_state_valid(current):
            self.current_path = None
            self.path_revision = None
            return "CURRENT_STATE_UNSAFE", None

        candidate = np.vstack((current[None, :], remaining))

        if self.is_path_valid(candidate):
            self.current_path = candidate.copy()
            self.path_revision = (
                self.collision_filter.environment_revision
            )
            return "UNCHANGED", candidate

        if self.plan(current, self.goal_q):
            return "REPLANNED", self.current_path.copy()

        return "NO_PATH", None


def path_distance(path):
    path = np.asarray(path, dtype=float)

    if len(path) < 2:
        return 0.0

    return float(
        np.linalg.norm(
            np.diff(path, axis=0), axis=1
        ).sum()
    )


def main():
    parser = argparse.ArgumentParser(
        description="CRA offline dynamic replanning test"
    )

    parser.add_argument("--graph", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--config", required=True)

    parser.add_argument(
        "--start-index", type=int, default=0
    )
    parser.add_argument(
        "--goal-index", type=int, default=100
    )
    parser.add_argument(
        "--obstacle", default="obstacle_1"
    )
    parser.add_argument(
        "--robot-geom", default="link3_visual"
    )
    parser.add_argument(
        "--current-waypoint", type=int, default=0
    )
    parser.add_argument(
        "--show-filter-stats", action="store_true"
    )

    args = parser.parse_args()

    planner = PathPlanner(
        graph_path=args.graph,
        model_path=args.model,
        config_path=args.config,
    )

    count = len(planner.joint_q)

    if not (
        0 <= args.start_index < count
        and 0 <= args.goal_index < count
    ):
        parser.error("Invalid graph index")

    start_q = planner.joint_q[args.start_index]
    goal_q = planner.joint_q[args.goal_index]

    print("CRA Dynamic Replanning Test")
    print(f"Joint states : {count}")
    print(f"Start index  : {args.start_index}")
    print(f"Goal index   : {args.goal_index}")

    if args.show_filter_stats:
        stats = planner.get_filter_statistics()

        print("\nINITIAL GRAPH FILTER")
        print(
            f"Valid nodes : "
            f"{stats['valid_nodes']}/{stats['total_nodes']}"
        )
        print(
            f"Valid edges : "
            f"{stats['valid_edges']}/{stats['total_edges']}"
        )

    if not planner.plan(start_q, goal_q):
        print("\nINITIAL PLANNING")
        print("Path found: NO")
        return

    initial_path = planner.current_path.copy()

    print("\nINITIAL PLANNING")
    print("Path found     : YES")
    print(f"Waypoints      : {len(initial_path)}")
    print(
        f"Joint distance : "
        f"{path_distance(initial_path):.4f} rad"
    )

    index = args.current_waypoint

    if not 0 <= index < len(initial_path):
        parser.error("Invalid current waypoint")

    current_q = initial_path[index]
    remaining = initial_path[index + 1:]

    blocked_index = min(
        len(initial_path) - 1,
        max(index + 1, len(initial_path) // 2),
    )

    blocked_q = initial_path[blocked_index]
    checker = planner.collision_filter

    geom_id = mujoco.mj_name2id(
        checker.model,
        mujoco.mjtObj.mjOBJ_GEOM,
        args.robot_geom,
    )

    if geom_id < 0:
        raise ValueError(
            f"Robot geom not found: {args.robot_geom}"
        )

    if not checker.is_state_valid(blocked_q):
        print("Selected blocked waypoint is already invalid")
        return

    obstacle_position = checker.data.geom_xpos[
        geom_id
    ].copy()

    planner.update_obstacle(
        name=args.obstacle,
        position=obstacle_position,
        size=[0.20, 0.05, 0.09],
    )

    print("\nOBSTACLE UPDATED")
    print(f"Obstacle : {args.obstacle}")
    print(f"Position : {obstacle_position.tolist()}")

    if args.show_filter_stats:
        stats = planner.get_filter_statistics()

        print("\nUPDATED GRAPH FILTER")
        print(
            f"Valid nodes : "
            f"{stats['valid_nodes']}/{stats['total_nodes']}"
        )
        print(
            f"Valid edges : "
            f"{stats['valid_edges']}/{stats['total_edges']}"
        )

    status, path = planner.replan_if_needed(
        current_q,
        remaining,
    )

    print("\nPATH REVALIDATION")
    print(f"Status : {status}")

    if path is None:
        print("No executable path available")
        print("Robot motion must remain stopped")
        return

    print(f"Waypoints      : {len(path)}")
    print(
        f"Joint distance : "
        f"{path_distance(path):.4f} rad"
    )
    print(
        "Local connections used: "
        f"{planner.planner.last_used_local_connections}"
    )
    print(
        "Path validation: "
        f"{'PASSED' if planner.is_path_valid(path) else 'FAILED'}"
    )
    print("No motion command was sent")


if __name__ == "__main__":
    main()