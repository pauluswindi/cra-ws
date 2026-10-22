#!/usr/bin/env python3

from pathlib import Path

import mujoco
import numpy as np

from workspace.frame_transform import FrameTransform


class CollisionChecker:

    def __init__(self, model_path, safety_margin=0.005):

        # Memuat model
        self.model = mujoco.MjModel.from_xml_path(
            str(Path(model_path).resolve())
        )
        self.data = mujoco.MjData(self.model)

        self.margin = float(safety_margin)
        self.joint_names = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5"
        ]

        self.joint_ids = np.array([
            mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name
            )
            for name in self.joint_names
        ], dtype=int)

        if np.any(self.joint_ids < 0):
            raise RuntimeError("Joint robot tidak ditemukan")

        self.qadr = self.model.jnt_qposadr[
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
            "base_link"
        )

        self.eof_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "end_effector"
        )

        if self.base_id < 0 or self.eof_id < 0:
            raise RuntimeError(
                "base_link atau end_effector tidak ditemukan"
            )

        # Mengidentifikasi geometri collision
        self.robot_geoms = set()
        self.obstacle_geoms = {}
        self.obstacle_bodies = {}

        for bid in range(self.model.nbody):

            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                bid
            ) or ""

            if name.startswith("obstacle_"):
                self.obstacle_bodies[name] = bid

        for gid in range(self.model.ngeom):

            bid = int(self.model.geom_bodyid[gid])
            body_name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                bid
            ) or ""

            if body_name.startswith("obstacle_"):

                self.obstacle_geoms[gid] = body_name

                self.model.geom_margin[gid] = max(
                    self.margin,
                    float(self.model.geom_margin[gid])
                )

                continue

            parent = bid

            while parent > 0:

                if parent == self.base_id:
                    self.robot_geoms.add(gid)
                    break

                parent = int(
                    self.model.body_parentid[parent]
                )

        # Menyimpan collision mask
        self.original_masks = {
            gid: (
                int(self.model.geom_contype[gid]),
                int(self.model.geom_conaffinity[gid])
            )
            for gid in self.obstacle_geoms
        }

        # Posisi obstacle aktif
        self.active_obstacles = {}

        mujoco.mj_forward(self.model, self.data)

    # Mengatur konfigurasi joint
    def set_q(self, q):

        q = np.asarray(q, dtype=float)

        if (
            q.shape != (5,)
            or not np.all(np.isfinite(q))
        ):
            raise ValueError("Konfigurasi joint tidak valid")

        self.data.qpos[self.qadr] = q
        self.data.qvel[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

    # Membaca posisi EoF pada frame base_link
    def get_eof(self, q):

        self.set_q(q)

        base_pos = np.asarray(
            self.data.xpos[self.base_id]
        )

        base_rot = np.asarray(
            self.data.xmat[self.base_id]
        ).reshape(3, 3)

        eof_world = np.asarray(
            self.data.xpos[self.eof_id]
        )

        eof_base = base_rot.T @ (
            eof_world - base_pos
        )

        return np.asarray(
            FrameTransform.mujoco_base_to_robot_base(
                eof_base
            ),
            dtype=float
        )

    # Membaca Jacobian posisi EoF
    def get_jacobian(self, q):

        self.set_q(q)

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        mujoco.mj_jacBody(
            self.model,
            self.data,
            jacp,
            jacr,
            self.eof_id
        )

        dof_indices = [
            int(self.model.jnt_dofadr[jid])
            for jid in self.joint_ids
        ]

        return jacp[:, dof_indices]

    # Mengonversi vektor posisi ke world MuJoCo
    def robot_to_world(self, position):

        self.set_q(
            self.data.qpos[self.qadr].copy()
        )

        base_pos = np.asarray(
            self.data.xpos[self.base_id]
        )

        base_rot = np.asarray(
            self.data.xmat[self.base_id]
        ).reshape(3, 3)

        local = (
            FrameTransform.robot_base_to_mujoco_base(
                np.asarray(position, dtype=float)
            )
        )

        return base_pos + base_rot @ local

    # Memperbarui obstacle dari marker ROS
    def set_obstacle(
        self,
        obstacle_id,
        position,
        size,
        quaternion
    ):

        name = f"obstacle_{int(obstacle_id)}"

        if name not in self.obstacle_bodies:
            raise ValueError(
                f"{name} tidak ditemukan pada XML"
            )

        bid = self.obstacle_bodies[name]
        mid = int(self.model.body_mocapid[bid])

        if mid < 0:
            raise ValueError(
                f"{name} harus mocap=true"
            )

        position = np.asarray(position, dtype=float)
        size = np.asarray(size, dtype=float)
        quaternion = np.asarray(quaternion, dtype=float)

        if (
            position.shape != (3,)
            or size.shape != (3,)
            or quaternion.shape != (4,)
            or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(size))
            or not np.all(np.isfinite(quaternion))
            or np.any(size <= 0)
        ):
            raise ValueError("Data obstacle tidak valid")

        if np.linalg.norm(quaternion) < 1e-9:
            raise ValueError("Quaternion obstacle tidak valid")

        quaternion /= np.linalg.norm(quaternion)

        # Transformasi posisi
        self.data.mocap_pos[mid] = (
            self.robot_to_world(position)
        )

        # Transformasi orientasi
        origin = (
            FrameTransform.robot_base_to_mujoco_base(
                np.zeros(3)
            )
        )

        basis = np.column_stack([
            FrameTransform.robot_base_to_mujoco_base(
                np.eye(3)[i]
            ) - origin
            for i in range(3)
        ])

        quat_mj = np.array([
            quaternion[3],
            quaternion[0],
            quaternion[1],
            quaternion[2]
        ])

        rot_flat = np.empty(9)
        mujoco.mju_quat2Mat(rot_flat, quat_mj)

        base_rot = np.asarray(
            self.data.xmat[self.base_id]
        ).reshape(3, 3)

        rot_world = (
            base_rot
            @ basis
            @ rot_flat.reshape(3, 3)
            @ basis.T
        )

        quat_world = np.empty(4)

        mujoco.mju_mat2Quat(
            quat_world,
            np.ascontiguousarray(
                rot_world
            ).reshape(9)
        )

        self.data.mocap_quat[mid] = quat_world

        # Memperbarui ukuran balok
        for gid, obstacle_name in self.obstacle_geoms.items():

            if obstacle_name != name:
                continue

            if int(self.model.geom_type[gid]) != int(
                mujoco.mjtGeom.mjGEOM_BOX
            ):
                raise ValueError(
                    "Versi ini memerlukan obstacle type=box"
                )

            self.model.geom_size[gid, :3] = size / 2

            contype, conaffinity = (
                self.original_masks[gid]
            )

            self.model.geom_contype[gid] = contype
            self.model.geom_conaffinity[gid] = conaffinity

        self.active_obstacles[name] = True

        mujoco.mj_forward(
            self.model,
            self.data
        )

    # Menghapus obstacle
    def remove_obstacle(self, obstacle_id):

        name = f"obstacle_{int(obstacle_id)}"

        if name not in self.obstacle_bodies:
            return

        bid = self.obstacle_bodies[name]
        mid = int(self.model.body_mocapid[bid])

        self.data.mocap_pos[mid] = [
            100.0, 100.0, 100.0
        ]

        for gid, obstacle_name in self.obstacle_geoms.items():

            if obstacle_name == name:
                self.model.geom_contype[gid] = 0
                self.model.geom_conaffinity[gid] = 0

        self.active_obstacles.pop(name, None)

        mujoco.mj_forward(
            self.model,
            self.data
        )

    # Menghapus semua obstacle
    def clear_obstacles(self):

        for name in list(self.active_obstacles):
            obstacle_id = int(
                name.rsplit("_", 1)[1]
            )
            self.remove_obstacle(obstacle_id)

    # Memeriksa batas joint
    def joint_valid(self, q):

        q = np.asarray(q, dtype=float)

        return bool(
            q.shape == (5,)
            and np.all(np.isfinite(q))
            and np.all(
                ~self.limited
                | (
                    (q >= self.lower)
                    & (q <= self.upper)
                )
            )
        )

    # Memeriksa seluruh link terhadap obstacle
    def check_configuration(self, q):

        if not self.joint_valid(q):
            return False, ["joint_limit"]

        self.set_q(q)

        collisions = []

        for contact in self.data.contact[:self.data.ncon]:

            a = int(contact.geom1)
            b = int(contact.geom2)

            if not (
                (
                    a in self.robot_geoms
                    and b in self.obstacle_geoms
                )
                or
                (
                    b in self.robot_geoms
                    and a in self.obstacle_geoms
                )
            ):
                continue

            if float(contact.dist) > self.margin:
                continue

            name_a = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                a
            )

            name_b = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                b
            )

            collisions.append({
                "geom1": name_a,
                "geom2": name_b,
                "distance": float(contact.dist)
            })

        return len(collisions) == 0, collisions

    # Memeriksa interpolasi joint
    def check_motion(
        self,
        q_start,
        q_end,
        max_step=0.02
    ):

        q_start = np.asarray(q_start, dtype=float)
        q_end = np.asarray(q_end, dtype=float)

        if not (
            self.joint_valid(q_start)
            and self.joint_valid(q_end)
            and max_step > 0
        ):
            return False, ["invalid_motion"]

        steps = max(
            1,
            int(np.ceil(
                np.max(np.abs(q_end - q_start))
                / max_step
            ))
        )

        for i in range(steps + 1):

            alpha = i / steps

            q = (
                q_start * (1 - alpha)
                + q_end * alpha
            )

            valid, reason = self.check_configuration(q)

            if not valid:
                return False, reason

        return True, []