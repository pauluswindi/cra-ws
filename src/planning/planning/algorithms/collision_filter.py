import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml


class CollisionFilter:
    def __init__(
        self,
        model_path,
        joint_names,
        safety_margin=0.005,
        max_joint_step=0.02,
    ):
        if not np.isfinite(safety_margin) or safety_margin < 0:
            raise ValueError("Invalid collision safety margin")

        if not np.isfinite(max_joint_step) or max_joint_step <= 0:
            raise ValueError("Invalid joint interpolation step")

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        self.safety_margin = float(safety_margin)
        self.max_joint_step = float(max_joint_step)
        self.environment_revision = 0

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

            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
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

        base_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "base_link",
        )

        if base_id < 0:
            raise ValueError("Robot body 'base_link' not found")

        robot_bodies = set()

        for body_id in range(self.model.nbody):
            current = body_id

            while current > 0:
                if current == base_id:
                    robot_bodies.add(body_id)
                    break

                current = int(self.model.body_parentid[current])

        self.robot_geoms = set()

        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])

            if body_id not in robot_bodies:
                continue

            if (
                int(self.model.geom_contype[geom_id]) == 0
                and int(self.model.geom_conaffinity[geom_id]) == 0
            ):
                continue

            self.robot_geoms.add(geom_id)

        if not self.robot_geoms:
            raise RuntimeError("No robot collision geoms found")

        # Preserve the original collision masks for obstacle updates.
        self.obstacle_geoms = {}

        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_id,
            )

            if not name:
                continue

            if name != "target" and not name.startswith("obstacle_"):
                continue

            mocap_id = int(self.model.body_mocapid[body_id])

            if mocap_id < 0:
                raise ValueError(f"Obstacle must be mocap: {name}")

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

        # Request contacts within the configured safety clearance.
        for geom_id in range(self.model.ngeom):
            self.model.geom_margin[geom_id] = max(
                float(self.model.geom_margin[geom_id]),
                self.safety_margin,
            )

        mujoco.mj_forward(self.model, self.data)

    def set_obstacle_pose(
        self,
        name,
        position,
        quaternion=None,
        size=None,
    ):
        if name not in self.obstacle_geoms:
            raise ValueError(f"Unknown obstacle: {name}")

        position = np.asarray(position, dtype=np.float64)

        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("Position must contain three finite values")

        if quaternion is None:
            quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            quaternion = np.asarray(quaternion, dtype=np.float64)

        if (
            quaternion.shape != (4,)
            or not np.isfinite(quaternion).all()
        ):
            raise ValueError("Quaternion must contain four finite values")

        norm = np.linalg.norm(quaternion)

        if norm < 1e-9:
            raise ValueError("Invalid obstacle quaternion")

        quaternion = quaternion / norm

        obstacle = self.obstacle_geoms[name]

        if size is not None:
            size = np.asarray(size, dtype=np.float64)

            if (
                size.shape != (3,)
                or not np.isfinite(size).all()
                or np.any(size <= 0)
            ):
                raise ValueError("Size must contain positive dimensions")

            for geom_id in obstacle["geom_ids"]:
                if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
                    raise ValueError(
                        f"Obstacle geometry is not a box: {name}"
                    )

            # MuJoCo stores box dimensions as half-extents.
            for geom_id in obstacle["geom_ids"]:
                self.model.geom_size[geom_id, :3] = size / 2.0

        mocap_id = obstacle["mocap_id"]

        self.data.mocap_pos[mocap_id] = position
        self.data.mocap_quat[mocap_id] = quaternion

        for geom_id, (contype, conaffinity) in obstacle["masks"].items():
            self.model.geom_contype[geom_id] = contype
            self.model.geom_conaffinity[geom_id] = conaffinity

        self.environment_revision += 1

        mujoco.mj_forward(self.model, self.data)

    def remove_obstacle(self, name):
        if name not in self.obstacle_geoms:
            raise ValueError(f"Unknown obstacle: {name}")

        if name == "target":
            raise ValueError("Target cannot be removed as an obstacle")

        obstacle = self.obstacle_geoms[name]

        mocap_id = obstacle["mocap_id"]

        self.data.mocap_pos[mocap_id] = [100.0, 100.0, 100.0]

        for geom_id in obstacle["geom_ids"]:
            self.model.geom_contype[geom_id] = 0
            self.model.geom_conaffinity[geom_id] = 0

        self.environment_revision += 1

        mujoco.mj_forward(self.model, self.data)

    def is_state_valid(self, joints):
        joints = np.asarray(joints, dtype=np.float64)

        if joints.shape != (5,) or not np.isfinite(joints).all():
            return False

        within_limits = (
            ~self.limited
            | ((joints >= self.lower) & (joints <= self.upper))
        )

        if not np.all(within_limits):
            return False

        self.data.qpos[self.qpos_addresses] = joints
        self.data.qvel[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

        for contact in self.data.contact:
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)

            if (
                geom1 not in self.robot_geoms
                and geom2 not in self.robot_geoms
            ):
                continue

            if float(contact.dist) <= self.safety_margin:
                return False

        return True

    def is_edge_valid(self, start, goal):
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)

        if start.shape != (5,) or goal.shape != (5,):
            return False

        if not np.isfinite(start).all() or not np.isfinite(goal).all():
            return False

        difference = goal - start

        steps = max(
            1,
            int(
                np.ceil(
                    np.max(np.abs(difference)) / self.max_joint_step
                )
            ),
        )

        for index in range(steps + 1):
            alpha = index / steps
            joints = start + alpha * difference

            if not self.is_state_valid(joints):
                return False

        return True

    def is_path_valid(self, joint_path):
        path = np.asarray(joint_path, dtype=np.float64)

        if path.ndim != 2 or path.shape[1] != 5 or len(path) == 0:
            return False

        for q in path:
            if not self.is_state_valid(q):
                return False

        for start, goal in zip(path[:-1], path[1:]):
            if not self.is_edge_valid(start, goal):
                return False

        return True

    def filter_graph(self, joints, edges):
        joints = np.asarray(joints, dtype=np.float64)
        edges = np.asarray(edges, dtype=np.int64)

        if joints.ndim != 2 or joints.shape[1] != 5:
            raise ValueError("Expected joint states with shape (N, 5)")

        if edges.ndim != 2 or edges.shape[1] != 2:
            raise ValueError("Expected joint edges with shape (E, 2)")

        if len(edges) and (
            edges.min() < 0 or edges.max() >= len(joints)
        ):
            raise ValueError("Joint edge index out of range")

        revision = self.environment_revision

        valid_states = np.array(
            [self.is_state_valid(q) for q in joints],
            dtype=bool,
        )

        valid_edges = np.zeros(len(edges), dtype=bool)

        for index, (a, b) in enumerate(edges):
            if not valid_states[a] or not valid_states[b]:
                continue

            valid_edges[index] = self.is_edge_valid(
                joints[a],
                joints[b],
            )

        if revision != self.environment_revision:
            raise RuntimeError(
                "Obstacle configuration changed during validation"
            )

        return valid_states, valid_edges


def main():
    parser = argparse.ArgumentParser(
        description="Validate CRA joint graph using MuJoCo"
    )

    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)

    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    with np.load(args.graph, allow_pickle=False) as data:
        joints = data["joint_q"].copy()
        edges = data["joint_edges"].copy()
        joint_names = data["joint_names"].tolist()

    collision_filter = CollisionFilter(
        model_path=args.model,
        joint_names=joint_names,
        safety_margin=float(
            config["planning"]["obstacle_safety_margin"]
        ),
    )

    print("CRA Whole-Arm Collision Filtering", flush=True)
    print(f"States       : {len(joints)}", flush=True)
    print(f"Edges        : {len(edges)}", flush=True)
    print(
        f"Safety margin: {collision_filter.safety_margin:.4f} m",
        flush=True,
    )

    valid_states, valid_edges = collision_filter.filter_graph(
        joints,
        edges,
    )

    print("\nCollision filtering completed", flush=True)
    print(f"Valid states   : {np.count_nonzero(valid_states)}")
    print(f"Invalid states : {np.count_nonzero(~valid_states)}")
    print(f"Valid edges    : {np.count_nonzero(valid_edges)}")
    print(f"Invalid edges  : {np.count_nonzero(~valid_edges)}")


if __name__ == "__main__":
    main()