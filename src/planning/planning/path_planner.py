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
            self.joint_eof = np.asarray(
                graph["joint_eof"], dtype=float
            ).copy()

        if len(self.joint_names) != 5:
            raise ValueError("Expected five robot joints")

        if (
            self.joint_q.ndim != 2
            or self.joint_q.shape[1] != 5
            or not np.isfinite(self.joint_q).all()
        ):
            raise ValueError("Invalid joint graph configurations")

        if (
            self.joint_eof.shape != (len(self.joint_q), 3)
            or not np.isfinite(self.joint_eof).all()
        ):
            raise ValueError("Invalid joint end-effector positions")

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

        self.max_joint_delta = float(
            joint_config["max_joint_delta"]
        )
        self.max_goal_candidates = int(joint_config.get("max_candidates", 4))
        if self.max_goal_candidates < 1:
            raise ValueError("max_candidates must be positive")

        self.max_segment_delta = float(
            config["motion"]["joint"]["max_segment_delta"]
        )

        if (
            not np.isfinite(self.max_segment_delta)
            or self.max_segment_delta <= 0.0
        ):
            raise ValueError("Invalid maximum joint segment delta")

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
            max_joint_delta=self.max_joint_delta,
        )

        self.current_path = None
        self.goal_q = None
        self.path_revision = None
        self.last_plan_failure = "NONE"
        self.last_human_retreat_failure = "NONE"
        self.safe_recovery_path = None

        self.valid_node_mask = None
        self.valid_edge_mask = None
        self.graph_filter_revision = None

    def human_data_ready(self):
        checker = self.collision_filter

        return (
            not checker.human_obstacles_enabled
            or checker.human_data_ready()
        )

    def select_goal(self, target, goal_tolerance, target_clearance):
        return next(self.goal_candidates(target, goal_tolerance, target_clearance), None)

    def goal_candidates(self, target, goal_tolerance, target_clearance):
        """Yield valid graph configurations in Cartesian-distance order."""
        if not self.human_data_ready():
            return

        target = np.asarray(target, dtype=float)

        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Invalid Cartesian target")

        if (
            not np.isfinite(goal_tolerance)
            or goal_tolerance <= 0.0
            or not np.isfinite(target_clearance)
            or not 0.0 <= target_clearance < goal_tolerance
        ):
            raise ValueError("Invalid goal selection tolerances")

        distances = np.linalg.norm(
            self.joint_eof - target, axis=1
        )

        for index in np.argsort(distances):
            distance = float(distances[index])

            if distance > goal_tolerance:
                break

            if distance < target_clearance:
                continue
            if not self.human_data_ready():
                return

            joints = self.joint_q[index]

            if self.collision_filter.is_state_valid(joints):
                yield joints.copy()

    def find_alternative_goal_path(self, start, target, failed_goal,
                                   goal_tolerance, target_clearance):
        """Try other valid joint goals for the same Cartesian target bounds.

        Return the successful joint goal, with its fully validated path stored
        as usual. Do not retry alternatives for unsafe input or expired data.
        """
        if self.last_plan_failure != "NO_GRAPH_PATH":
            return None
        revision = self.collision_filter.environment_revision
        candidates = self.goal_candidates(target, goal_tolerance, target_clearance)
        tried = [np.asarray(failed_goal).copy()]
        for goal in candidates:
            if not any(np.array_equal(goal, previous) for previous in tried):
                if len(tried) >= self.max_goal_candidates:
                    break
                tried.append(goal.copy())
                if self.collision_filter.environment_revision != revision:
                    self._plan_failed("ENVIRONMENT_CHANGED", "environment changed between goals")
                    return None
                print(f"[PathPlanner] Trying Cartesian goal candidate {len(tried)}/"
                      f"{self.max_goal_candidates}", flush=True)
                if self.plan(start, goal):
                    return goal.copy()
                if self.last_plan_failure != "NO_GRAPH_PATH":
                    return None
        if self.collision_filter.environment_revision != revision:
            self._plan_failed("ENVIRONMENT_CHANGED", "environment changed between goals")
        elif not self.human_data_ready():
            self._plan_failed("HUMAN_VISION_UNKNOWN", "human data expired between goal candidates")
        return None

    def prepare_execution_path(self, path):
        if not self.human_data_ready():
            raise ValueError("Human Vision data unavailable")

        path = np.asarray(path, dtype=float)

        if (
            path.ndim != 2
            or path.shape[1] != 5
            or len(path) < 2
            or not np.isfinite(path).all()
        ):
            raise ValueError("Invalid execution path")

        result = [path[0].copy()]

        for goal in path[1:]:
            start = result[-1]
            delta = goal - start
            max_delta = float(np.max(np.abs(delta)))

            if max_delta <= 1e-9:
                continue

            steps = max(
                1,
                int(np.ceil(max_delta / self.max_segment_delta))
            )

            if len(result) + steps > 10000:
                raise ValueError(
                    "Execution path exceeds waypoint limit"
                )

            for step in range(1, steps + 1):
                result.append(
                    start + delta * (step / steps)
                )

        if len(result) < 2:
            raise ValueError("Execution path has no movement")

        result = np.asarray(result)

        if not self.is_path_valid(result):
            raise ValueError(
                "Execution path failed collision validation"
            )

        return result

    def invalidate_graph_filter(self):
        self.valid_node_mask = None
        self.valid_edge_mask = None
        self.graph_filter_revision = None

    def filter_joint_graph(self):
        if not self.human_data_ready():
            raise RuntimeError("Human Vision data unavailable")

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
        if not self.human_data_ready():
            return False

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

    def find_nearest_safe_configuration(self, current_q):
        current = np.asarray(current_q, dtype=float)
        checker = self.collision_filter

        if (
            current.shape != (5,)
            or not np.isfinite(current).all()
            or not checker.is_state_valid(current)
        ):
            return None

        distances = np.linalg.norm(
            self.joint_q - current, axis=1
        )

        for index in np.argsort(distances):
            candidate = self.joint_q[index]

            if (
                np.max(np.abs(candidate - current))
                > self.max_joint_delta
            ):
                continue

            if (
                checker.is_state_valid(candidate)
                and checker.is_edge_valid(current, candidate)
            ):
                return candidate.copy()

        return None

    def find_safe_recovery_path(self, current_q, max_steps=12):
        """Find a small monotonic contact retreat, including initial contact."""

        self.safe_recovery_path = None

        checker = self.collision_filter
        current = np.asarray(current_q, dtype=float)

        if current.shape != (5,) or not np.isfinite(current).all():
            return None

        revision = checker.environment_revision
        initial = checker.recovery_contact_state(current)

        if initial is None or initial["status"] == "CLEAR":
            return None

        step_size = min(
            checker.max_joint_step,
            self.max_segment_delta,
            0.01,
        )

        path = [current.copy()]
        current_state = initial

        for _ in range(max_steps):
            best = None
            best_state = None
            best_improvement = 1e-5

            for joint in range(5):
                for sign in (-1.0, 1.0):
                    candidate = path[-1].copy()
                    candidate[joint] += sign * step_size

                    if checker.limited[joint] and not (
                        checker.lower[joint]
                        <= candidate[joint]
                        <= checker.upper[joint]
                    ):
                        continue

                    if not checker.is_recovery_transition_valid(
                        path[-1],
                        candidate,
                        current_state,
                    ):
                        continue

                    state = checker.recovery_contact_state(candidate)

                    if state is None:
                        continue

                    old_deficit = sum(
                        checker.safety_margin - distance
                        for distance in current_state["pairs"].values()
                    )

                    new_deficit = sum(
                        checker.safety_margin - distance
                        for distance in state["pairs"].values()
                    )

                    improvement = old_deficit - new_deficit

                    if improvement > best_improvement:
                        best = candidate
                        best_state = state
                        best_improvement = improvement

            if checker.environment_revision != revision or best is None:
                return None

            path.append(best)
            current_state = best_state

            if current_state["status"] == "CLEAR":
                result = np.asarray(path, dtype=float)

                if (
                    checker.environment_revision != revision
                    or not checker.is_state_valid(best)
                ):
                    return None

                self.safe_recovery_path = result.copy()
                return result

        return None

    def find_human_margin_retreat(self, current_q, max_steps=32):
        """Find a bounded retreat away from a human, without contact.

        Return only a certified straight joint segment, so execution can be
        revalidated from measured feedback without inferring waypoint progress.
        Prefer leaving the margin; allow certified progress in stages when the
        search budget cannot reach it in a single segment.
        """
        checker = self.collision_filter
        self.last_human_retreat_failure = "NONE"
        start = np.asarray(current_q, dtype=float)
        initial = checker.human_margin_state(start)
        if initial is None or initial["status"] != "NEAR":
            diagnostic = checker.diagnose_state(start)
            self.last_human_retreat_failure = diagnostic["reason"]
            if diagnostic["reason"] == "VALID":
                self.last_human_retreat_failure = "NOT_IN_HUMAN_MARGIN"
            if (
                diagnostic["reason"] == "HUMAN_COLLISION"
                and diagnostic["human"]["clearance"] <= 0.
            ):
                self.last_human_retreat_failure = "HUMAN_OVERLAP"
            return None
        revision = checker.environment_revision
        current = start.copy()
        state = initial
        step = min(0.01, checker.max_joint_step, self.max_segment_delta)
        target = checker.human_safety_margin + checker.safety_margin
        for _ in range(max_steps):
            best, best_state = None, None
            improvement = 1e-6
            for joint in range(5):
                for sign in (-1., 1.):
                    candidate = current.copy()
                    candidate[joint] += sign * step
                    if np.max(np.abs(candidate - start)) > min(
                        self.max_joint_delta, self.max_segment_delta
                    ):
                        continue
                    candidate_state = checker.human_margin_state(candidate)
                    if candidate_state is None:
                        if (
                            not checker.human_data_ready()
                            or checker.environment_revision != revision
                        ):
                            self.last_human_retreat_failure = (
                                "HUMAN_DATA_UNKNOWN" if not checker.human_data_ready()
                                else "ENVIRONMENT_CHANGED"
                            )
                            return None
                        # Invalid joints or static contact disqualify only this
                        # candidate, just as the full transition checker would.
                        continue
                    gain = candidate_state["clearance"] - state["clearance"]
                    if gain <= improvement:
                        continue
                    # A worsening endpoint cannot pass the sampled monotonic
                    # certificate. Screen it before evaluating the whole edge.
                    if candidate_state["pairs"].keys() != state["pairs"].keys() or any(
                        value < min(state["pairs"][pair], checker.human_safety_margin) - 1e-9
                        for pair, value in candidate_state["pairs"].items()
                    ):
                        continue
                    if checker.is_human_margin_transition_valid(current, candidate):
                        best, best_state, improvement = candidate, candidate_state, gain
            if (
                best is None or checker.environment_revision != revision
                or not checker.human_data_ready()
            ):
                self.last_human_retreat_failure = (
                    "HUMAN_DATA_UNKNOWN" if not checker.human_data_ready()
                    else "ENVIRONMENT_CHANGED" if checker.environment_revision != revision
                    else "NO_MONOTONIC_STEP"
                )
                return None
            current, state = best, best_state
            if (
                state["clearance"] > target
                and checker.is_state_valid(current)
                and checker.is_human_margin_transition_valid(start, current)
                and checker.environment_revision == revision
            ):
                return np.vstack((start, current))
        if (
            state["clearance"] - initial["clearance"] >= checker.safety_margin
            and checker.is_human_margin_transition_valid(start, current)
            and checker.environment_revision == revision
            and checker.human_data_ready()
        ):
            return np.vstack((start, current))
        self.last_human_retreat_failure = (
            "HUMAN_DATA_UNKNOWN" if not checker.human_data_ready()
            else "ENVIRONMENT_CHANGED" if checker.environment_revision != revision
            else "SEARCH_BUDGET_EXHAUSTED"
        )
        return None

    def find_alternative_path(self, start, goal, revision):
        checker = self.collision_filter

        distances = np.linalg.norm(
            self.joint_q - start, axis=1
        )

        attempts = 0

        for index in np.argsort(distances):
            if (
                checker.environment_revision != revision
                or not self.human_data_ready()
            ):
                return None
            intermediate = self.joint_q[index]

            if (
                np.max(np.abs(intermediate - start))
                > self.max_joint_delta
            ):
                continue

            if not checker.is_state_valid(intermediate):
                continue

            if not checker.is_edge_valid(start, intermediate):
                continue

            attempts += 1

            remainder = self.planner.plan(
                intermediate,
                goal,
            )

            if remainder is not None:
                path = np.vstack(
                    (start, np.asarray(remainder))
                )

                if (
                    checker.environment_revision == revision
                    and self.is_path_valid(path)
                ):
                    return path

            if (
                checker.environment_revision != revision
                or attempts >= 8
            ):
                break

        return None

    def _plan_failed(self, reason, message, joints=None):
        self.last_plan_failure = reason
        if reason == "HUMAN_VISION_UNKNOWN":
            # Expired observations can leave negative entries in A*'s caches
            # even when the next fresh observation has identical geometry.
            self.planner.revision = None

        if joints is not None:
            try:
                diagnostic = self.collision_filter.diagnose_state(joints)
            except Exception as error:
                diagnostic = {
                    "reason": "DIAGNOSTIC_ERROR",
                    "error": f"{type(error).__name__}: {error}",
                }
            message += (
                f"; q={joints.tolist()}, "
                f"revision={self.collision_filter.environment_revision}, "
                f"diagnostic={diagnostic}"
            )

        print(
            f"[PathPlanner] NO_PATH: {message}",
            flush=True,
        )

        return False

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
                "Invalid start or goal configuration"
            )

        self.current_path = None
        self.path_revision = None
        self.goal_q = goal.copy()
        self.last_plan_failure = "NONE"

        checker = self.collision_filter
        revision = checker.environment_revision

        if not self.human_data_ready():
            return self._plan_failed(
                "HUMAN_VISION_UNKNOWN",
                "Human Vision data unavailable",
            )

        if not checker.is_state_valid(start):
            return self._plan_failed(
                "START_INVALID",
                "start configuration is invalid",
                joints=start,
            )

        if not checker.is_state_valid(goal):
            return self._plan_failed(
                "GOAL_INVALID",
                "goal configuration is invalid",
                joints=goal,
            )

        if checker.environment_revision != revision:
            return self._plan_failed(
                "ENVIRONMENT_CHANGED",
                "environment changed",
            )

        path = self.planner.plan(start, goal)

        if not self.human_data_ready():
            return self._plan_failed(
                "HUMAN_VISION_UNKNOWN",
                "Human Vision data expired during graph search",
            )

        if path is None:
            path = self.find_alternative_path(
                start,
                goal,
                revision,
            )

        if not self.human_data_ready():
            return self._plan_failed(
                "HUMAN_VISION_UNKNOWN",
                "Human Vision data expired during alternative search",
            )

        if path is None:
            if checker.environment_revision != revision:
                return self._plan_failed(
                    "ENVIRONMENT_CHANGED",
                    "environment changed",
                )

            return self._plan_failed(
                "NO_GRAPH_PATH",
                "A* could not connect start to goal",
            )

        valid = (
            checker.environment_revision == revision
            and self.is_path_valid(path)
        )
        if not self.human_data_ready():
            return self._plan_failed(
                "HUMAN_VISION_UNKNOWN",
                "Human Vision data expired during final path validation",
            )
        if not valid:
            return self._plan_failed(
                "PATH_INVALID",
                "final path validation failed",
            )

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
        if not self.human_data_ready():
            return "HUMAN_VISION_UNKNOWN", None

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

        candidate = np.vstack(
            (current[None, :], remaining)
        )

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
            np.diff(path, axis=0),
            axis=1,
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
        "--start-index",
        type=int,
        default=0,
    )

    parser.add_argument(
        "--goal-index",
        type=int,
        default=100,
    )

    parser.add_argument(
        "--obstacle",
        default="obstacle_1",
    )

    parser.add_argument(
        "--robot-geom",
        default="link3_visual",
    )

    parser.add_argument(
        "--current-waypoint",
        type=int,
        default=0,
    )

    parser.add_argument(
        "--show-filter-stats",
        action="store_true",
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
