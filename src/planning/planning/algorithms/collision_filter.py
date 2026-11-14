#!/usr/bin/env python3

import time

import mujoco
import numpy as np

from planning.algorithms.human_collision import (
    nearest_human_collision,
    parse_human_snapshot,
    point_segment_distance,
)


class CollisionFilter:
    def __init__(
        self,
        model_path,
        joint_names,
        safety_margin=0.005,
        max_joint_step=0.02,
        human_safety_margin=0.10,
        human_data_timeout=0.5,
    ):
        for name, value, minimum in (
            ("safety_margin", safety_margin, 0.0),
            ("max_joint_step", max_joint_step, 0.0),
            ("human_safety_margin", human_safety_margin, 0.0),
            ("human_data_timeout", human_data_timeout, 0.0),
        ):
            if not np.isfinite(value) or value < minimum or (
                name in ("max_joint_step", "human_data_timeout")
                and value == 0
            ):
                raise ValueError(f"Invalid {name}")

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        self.safety_margin = float(safety_margin)
        self.max_joint_step = float(max_joint_step)
        self.human_safety_margin = float(human_safety_margin)
        self.human_data_timeout = float(human_data_timeout)
        self.environment_revision = 0

        self.human_obstacles_enabled = False
        self.human_status = "UNKNOWN"
        self.human_capsules = ()
        self.human_received_at = None
        self.human_expires_at = None

        self.joint_names = list(joint_names)
        self.joint_ids = []

        for name in self.joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            if joint_id < 0:
                raise ValueError(f"Joint not found: {name}")

            if (
                self.model.jnt_type[joint_id]
                != mujoco.mjtJoint.mjJNT_HINGE
            ):
                raise ValueError(f"Expected hinge joint: {name}")

            self.joint_ids.append(joint_id)

        if len(self.joint_ids) != 5:
            raise ValueError("Expected five arm joints")

        self.qpos_addresses = self.model.jnt_qposadr[
            self.joint_ids
        ].astype(int)

        self.lower = self.model.jnt_range[
            self.joint_ids, 0
        ].copy()

        self.upper = self.model.jnt_range[
            self.joint_ids, 1
        ].copy()

        self.limited = self.model.jnt_limited[
            self.joint_ids
        ].astype(bool)

        self.base_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "base_link",
        )

        if self.base_id < 0:
            raise ValueError("Robot body 'base_link' not found")

        robot_bodies = set()

        for body_id in range(self.model.nbody):
            current = body_id

            while current > 0:
                if current == self.base_id:
                    robot_bodies.add(body_id)
                    break

                current = int(
                    self.model.body_parentid[current]
                )

        self.robot_geoms = set()

        for geom_id in range(self.model.ngeom):
            body_id = int(
                self.model.geom_bodyid[geom_id]
            )

            if body_id not in robot_bodies:
                continue

            if (
                int(self.model.geom_contype[geom_id]) == 0
                and int(self.model.geom_conaffinity[geom_id]) == 0
            ):
                continue

            self.robot_geoms.add(geom_id)

        if not self.robot_geoms:
            raise RuntimeError(
                "No robot collision geoms found"
            )

        self.obstacle_geoms = {}

        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_id,
            )

            if not name:
                continue

            if (
                name != "target"
                and not name.startswith("obstacle_")
            ):
                continue

            mocap_id = int(
                self.model.body_mocapid[body_id]
            )

            if mocap_id < 0:
                raise ValueError(
                    f"Obstacle must be mocap: {name}"
                )

            geom_ids = [
                geom_id
                for geom_id in range(self.model.ngeom)
                if int(self.model.geom_bodyid[geom_id]) == body_id
            ]

            self.obstacle_geoms[name] = {
                "mocap_id": mocap_id,
                "geom_ids": geom_ids,
                "masks": {
                    geom_id: (
                        int(self.model.geom_contype[geom_id]),
                        int(self.model.geom_conaffinity[geom_id]),
                    )
                    for geom_id in geom_ids
                },
            }

        for geom_id in range(self.model.ngeom):
            self.model.geom_margin[geom_id] = max(
                float(self.model.geom_margin[geom_id]),
                self.safety_margin,
            )

        mujoco.mj_forward(self.model, self.data)

    def enable_human_obstacles(self, enabled):
        enabled = bool(enabled)

        if enabled != self.human_obstacles_enabled:
            self.human_obstacles_enabled = enabled
            self.environment_revision += 1

    def set_human_capsules(self, snapshot):
        """Store a validated HumanSnapshot, not a raw ROS message."""

        status, capsules = parse_human_snapshot(snapshot)

        changed = (
            status != self.human_status
            or capsules != self.human_capsules
        )

        self.human_status = status
        self.human_capsules = capsules

        self.human_received_at = (
            time.monotonic()
            if status == "OBSERVED"
            else None
        )
        self.human_expires_at = getattr(snapshot, "expires_at", None)

        if changed and self.human_obstacles_enabled:
            self.environment_revision += 1

        return status == "OBSERVED"

    def human_data_ready(self):
        now = time.monotonic()
        return (
            self.human_status == "OBSERVED"
            and self.human_received_at is not None
            and now - self.human_received_at
            <= self.human_data_timeout
            and (
                self.human_expires_at is None
                or now <= self.human_expires_at
            )
        )

    @staticmethod
    def _point_segment_distance(point, start, end):
        # Retained for compatibility with existing tooling.
        return point_segment_distance(point, start, end)

    def _human_collision_after_forward(self, include_pairs=False):
        base_position = self.data.xpos[self.base_id]
        base_rotation = self.data.xmat[self.base_id].reshape(3, 3)

        # MuJoCo positions are converted to the local MuJoCo base frame here.
        # human_collision.py then converts them to the CRA robot base_link frame.
        geoms = (
            (
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
                ),
                base_rotation.T @ (
                    self.data.geom_xpos[geom_id] - base_position
                ),
                self.model.geom_rbound[geom_id],
            )
            for geom_id in sorted(self.robot_geoms)
        )

        return nearest_human_collision(
            geoms, self.human_capsules, self.human_safety_margin,
            include_pairs=include_pairs,
        )

    def human_collision_report(self, joints):
        if not self.human_data_ready():
            return {
                "status": "UNKNOWN",
                "clearance": None,
                "reason": "HUMAN_DATA_UNKNOWN",
            }

        if not self._set_joint_configuration(joints):
            return {
                "status": "UNKNOWN",
                "clearance": None,
                "reason": "INVALID_JOINTS",
            }

        return self._human_collision_after_forward()

    def diagnose_state(self, joints):
        """Explain why a state is rejected; intended for on-demand debugging.

        This does not replace the fast boolean validity check used by A*.
        """
        if self.human_obstacles_enabled and not self.human_data_ready():
            return {"valid": False, "reason": "HUMAN_DATA_UNKNOWN"}

        if not self._set_joint_configuration(joints):
            return {"valid": False, "reason": "INVALID_JOINTS"}

        contacts = self.contact_report(joints)["contacts"]
        if contacts:
            closest = min(contacts, key=lambda item: item["distance"])
            return {
                "valid": False,
                "reason": "MUJOCO_CONTACT",
                "geom1": closest["geom1"],
                "geom2": closest["geom2"],
                "distance": closest["distance"],
            }

        if self.human_obstacles_enabled:
            human = self._human_collision_after_forward()
            if human["status"] != "FAR":
                return {
                    "valid": False,
                    "reason": "HUMAN_COLLISION" if human["status"] == "NEAR"
                    else "HUMAN_GEOMETRY_UNKNOWN",
                    "human": human,
                }

        return {"valid": True, "reason": "VALID"}

    def _set_joint_configuration(self, joints):
        joints = np.asarray(
            joints,
            dtype=np.float64,
        )

        if (
            joints.shape != (5,)
            or not np.isfinite(joints).all()
        ):
            return False

        within_limits = (
            ~self.limited
            | (
                (joints >= self.lower)
                & (joints <= self.upper)
            )
        )

        if not np.all(within_limits):
            return False

        self.data.qpos[self.qpos_addresses] = joints
        self.data.qvel[:] = 0.0

        mujoco.mj_forward(self.model, self.data)
        return True

    def set_obstacle_pose(
        self,
        name,
        position,
        quaternion=None,
        size=None,
    ):
        if name not in self.obstacle_geoms:
            raise ValueError(
                f"Unknown obstacle: {name}"
            )

        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if (
            position.shape != (3,)
            or not np.isfinite(position).all()
        ):
            raise ValueError(
                "Position must contain three finite values"
            )

        if quaternion is None:
            quaternion = np.array(
                [1.0, 0.0, 0.0, 0.0]
            )
        else:
            quaternion = np.asarray(
                quaternion,
                dtype=np.float64,
            )

        if (
            quaternion.shape != (4,)
            or not np.isfinite(quaternion).all()
        ):
            raise ValueError(
                "Quaternion must contain four finite values"
            )

        norm = np.linalg.norm(quaternion)

        if norm < 1e-9:
            raise ValueError(
                "Invalid obstacle quaternion"
            )

        quaternion = quaternion / norm
        obstacle = self.obstacle_geoms[name]

        if size is not None:
            size = np.asarray(
                size,
                dtype=np.float64,
            )

            if (
                size.shape != (3,)
                or not np.isfinite(size).all()
                or np.any(size <= 0)
            ):
                raise ValueError(
                    "Size must contain positive dimensions"
                )

            for geom_id in obstacle["geom_ids"]:
                if (
                    self.model.geom_type[geom_id]
                    != mujoco.mjtGeom.mjGEOM_BOX
                ):
                    raise ValueError(
                        f"Obstacle geometry is not a box: {name}"
                    )

            for geom_id in obstacle["geom_ids"]:
                self.model.geom_size[
                    geom_id, :3
                ] = size / 2.0

        mocap_id = obstacle["mocap_id"]

        self.data.mocap_pos[mocap_id] = position
        self.data.mocap_quat[mocap_id] = quaternion

        for geom_id, (
            contype,
            conaffinity,
        ) in obstacle["masks"].items():
            self.model.geom_contype[geom_id] = contype
            self.model.geom_conaffinity[geom_id] = conaffinity

        self.environment_revision += 1

        mujoco.mj_forward(self.model, self.data)

    def remove_obstacle(self, name):
        if name not in self.obstacle_geoms:
            raise ValueError(
                f"Unknown obstacle: {name}"
            )

        if name == "target":
            raise ValueError(
                "Target cannot be removed as an obstacle"
            )

        obstacle = self.obstacle_geoms[name]

        self.data.mocap_pos[
            obstacle["mocap_id"]
        ] = [100.0, 100.0, 100.0]

        for geom_id in obstacle["geom_ids"]:
            self.model.geom_contype[geom_id] = 0
            self.model.geom_conaffinity[geom_id] = 0

        self.environment_revision += 1

        mujoco.mj_forward(self.model, self.data)

    def is_state_valid(self, joints):
        if (
            self.human_obstacles_enabled
            and not self.human_data_ready()
        ):
            return False

        if not self._set_joint_configuration(joints):
            return False

        for contact in self.data.contact:
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)

            if (
                geom1 not in self.robot_geoms
                and geom2 not in self.robot_geoms
            ):
                continue

            if (
                float(contact.dist)
                <= self.safety_margin
            ):
                return False

        if self.human_obstacles_enabled:
            report = self._human_collision_after_forward()

            if report["status"] != "FAR":
                return False

        return True

    def contact_report(self, joints):
        """Report MuJoCo contacts only; not a global distance query."""

        if not self._set_joint_configuration(joints):
            return {
                "status": "INVALID_INPUT",
                "contacts": [],
            }

        contacts = []

        for contact in self.data.contact:
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)

            if (
                geom1 not in self.robot_geoms
                and geom2 not in self.robot_geoms
            ):
                continue

            distance = float(contact.dist)

            if distance > self.safety_margin:
                continue

            contacts.append({
                "geom1": mujoco.mj_id2name(
                    self.model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    geom1,
                ),
                "geom2": mujoco.mj_id2name(
                    self.model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    geom2,
                ),
                "distance": distance,
                "geom_ids": (geom1, geom2),
                "position": np.asarray(
                    contact.pos,
                    dtype=float,
                ).copy(),
                "normal": np.asarray(
                    contact.frame[:3],
                    dtype=float,
                ).copy(),
            })

        if any(
            item["distance"] < 0
            for item in contacts
        ):
            status = "PENETRATION"

        elif any(
            item["distance"] <= 0
            for item in contacts
        ):
            status = "CONTACT"

        elif contacts:
            status = "SAFETY_MARGIN"

        else:
            status = "CLEAR"

        return {
            "status": status,
            "contacts": contacts,
        }

    def recovery_contact_state(self, joints):
        report = self.contact_report(joints)

        if report["status"] in (
            "INVALID_INPUT",
            "JOINT_LIMIT",
        ):
            return None

        pairs = {}

        for contact in report["contacts"]:
            key = tuple(
                sorted(contact["geom_ids"])
            )

            pairs[key] = min(
                pairs.get(
                    key,
                    float("inf"),
                ),
                contact["distance"],
            )

        return {
            "status": report["status"],
            "pairs": pairs,
            "min_distance": min(
                pairs.values(),
                default=self.safety_margin,
            ),
        }

    def human_margin_state(self, joints):
        """Describe a non-contact state, including every robot/human pair.

        Margin retreat may leave an existing human safety buffer. It never
        permits penetration of either the human approximation or MuJoCo geoms.
        """
        if not self.human_obstacles_enabled or not self.human_data_ready():
            return None
        revision = self.environment_revision
        if self.contact_report(joints)["status"] != "CLEAR":
            return None
        report = self._human_collision_after_forward(include_pairs=True)
        if (
            report["status"] == "UNKNOWN" or report["clearance"] <= 0.0
            or not self.human_data_ready() or self.environment_revision != revision
        ):
            return None
        return report

    def is_human_margin_transition_valid(self, start, goal):
        """Certify non-worsening clearance for ALL initially close pairs.

        Pairs outside the margin must stay outside. Initially close pairs
        must never get closer and must not re-enter the buffer after exiting.
        Ordinary is_state_valid/is_edge_valid retain the original margins.
        """
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if (
            start.shape != (5,) or goal.shape != (5,)
            or not np.isfinite(start).all() or not np.isfinite(goal).all()
        ):
            return False
        revision = self.environment_revision
        previous = self.human_margin_state(start)
        if previous is None:
            return False
        steps = max(2, int(np.ceil(
            np.max(np.abs(goal - start)) / min(self.max_joint_step, 0.005)
        )))
        for index in range(1, steps + 1):
            state = self.human_margin_state(start + (goal - start) * (index / steps))
            if (
                state is None or self.environment_revision != revision
                or state["pairs"].keys() != previous["pairs"].keys()
            ):
                return False
            for pair, clearance in state["pairs"].items():
                floor = min(previous["pairs"][pair], self.human_safety_margin)
                if clearance < floor - 1e-9:
                    return False
            previous = state
        return self.human_data_ready() and self.environment_revision == revision

    def is_recovery_transition_valid(
        self,
        start,
        goal,
        initial_state=None,
    ):
        # Contact retreat is not validated around moving humans.
        if self.human_obstacles_enabled:
            return False

        start = np.asarray(
            start,
            dtype=float,
        )

        goal = np.asarray(
            goal,
            dtype=float,
        )

        if (
            start.shape != (5,)
            or goal.shape != (5,)
            or not np.isfinite(start).all()
            or not np.isfinite(goal).all()
        ):
            return False

        revision = self.environment_revision

        previous = (
            initial_state
            if initial_state is not None
            else self.recovery_contact_state(start)
        )

        if previous is None:
            return False

        original_pairs = set(
            previous["pairs"]
        )

        steps = max(
            2,
            int(
                np.ceil(
                    np.max(np.abs(goal - start))
                    / min(self.max_joint_step, 0.005)
                )
            ),
        )

        for index in range(1, steps + 1):
            sample = (
                start
                + (goal - start) * (index / steps)
            )

            state = self.recovery_contact_state(
                sample
            )

            if (
                state is None
                or self.environment_revision != revision
            ):
                return False

            if any(
                key not in original_pairs
                for key in state["pairs"]
            ):
                return False

            for key in original_pairs:
                before = previous["pairs"].get(
                    key,
                    self.safety_margin,
                )

                after = state["pairs"].get(
                    key,
                    self.safety_margin,
                )

                if after + 1e-5 < before:
                    return False

            previous = state

        return True

    def is_edge_valid(self, start, goal):
        start = np.asarray(
            start,
            dtype=np.float64,
        )

        goal = np.asarray(
            goal,
            dtype=np.float64,
        )

        if (
            start.shape != (5,)
            or goal.shape != (5,)
        ):
            return False

        if (
            not np.isfinite(start).all()
            or not np.isfinite(goal).all()
        ):
            return False

        difference = goal - start

        steps = max(
            1,
            int(
                np.ceil(
                    np.max(np.abs(difference))
                    / self.max_joint_step
                )
            ),
        )

        revision = self.environment_revision

        for index in range(steps + 1):
            joints = (
                start
                + difference * (index / steps)
            )

            if not self.is_state_valid(joints):
                return False

            if self.environment_revision != revision:
                return False

        return True

    def is_path_valid(self, joint_path):
        path = np.asarray(
            joint_path,
            dtype=np.float64,
        )

        if (
            path.ndim != 2
            or path.shape[1] != 5
            or len(path) == 0
        ):
            return False

        revision = self.environment_revision

        for q in path:
            if not self.is_state_valid(q):
                return False

        for start, goal in zip(
            path[:-1],
            path[1:],
        ):
            if not self.is_edge_valid(
                start,
                goal,
            ):
                return False

        return (
            self.environment_revision == revision
        )

    def filter_graph(self, joints, edges):
        joints = np.asarray(
            joints,
            dtype=np.float64,
        )

        edges = np.asarray(
            edges,
            dtype=np.int64,
        )

        if (
            joints.ndim != 2
            or joints.shape[1] != 5
        ):
            raise ValueError(
                "Expected joint states with shape (N, 5)"
            )

        if (
            edges.ndim != 2
            or edges.shape[1] != 2
        ):
            raise ValueError(
                "Expected joint edges with shape (E, 2)"
            )

        if len(edges) and (
            edges.min() < 0
            or edges.max() >= len(joints)
        ):
            raise ValueError(
                "Joint edge index out of range"
            )

        revision = self.environment_revision

        valid_states = np.array(
            [
                self.is_state_valid(q)
                for q in joints
            ],
            dtype=bool,
        )

        valid_edges = np.zeros(
            len(edges),
            dtype=bool,
        )

        for index, (a, b) in enumerate(edges):
            if (
                valid_states[a]
                and valid_states[b]
            ):
                valid_edges[index] = (
                    self.is_edge_valid(
                        joints[a],
                        joints[b],
                    )
                )

        if revision != self.environment_revision:
            raise RuntimeError(
                "Obstacle configuration changed during validation"
            )

        return valid_states, valid_edges


def main():
    # Preserve the existing `collision_filter` console entry point.
    from planning.algorithms.validate_joint_graph import main as cli_main

    cli_main()


if __name__ == "__main__":
    main()
