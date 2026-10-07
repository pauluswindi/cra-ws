#!/usr/bin/env python3

from pathlib import Path

import mujoco
import numpy as np


class RobotKinematics:
    """Kinematics and joint information for the robot."""

    def __init__(self, model_path):
        self.model_path = Path(model_path).resolve()

        if not self.model_path.exists():
            raise FileNotFoundError(
                f"MuJoCo model not found:\n{self.model_path}"
            )

        self.model = mujoco.MjModel.from_xml_path(
            str(self.model_path)
        )
        self.data = mujoco.MjData(self.model)

        self._initialize_body_ids()
        self._initialize_joint_info()
        self._initialize_actuator_info()

        mujoco.mj_forward(
            self.model,
            self.data,
        )

    # Body information

    def _initialize_body_ids(self):
        self.body_names = [
            "base_link",
            "link1_1",
            "link2_1",
            "link3_1",
            "link4_1",
            "link5_1",
            "end_effector",
        ]

        self.body_ids = {
            name: self.model.body(name).id
            for name in self.body_names
        }

    # Joint information

    def _initialize_joint_info(self):
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
        ]

        if len(self.joint_names) != self.model.nq:
            raise RuntimeError(
                f"Expected {len(self.joint_names)} joints, "
                f"but MuJoCo reports nq={self.model.nq}."
            )

        if len(self.joint_names) != self.model.nv:
            raise RuntimeError(
                f"Expected {len(self.joint_names)} DOFs, "
                f"but MuJoCo reports nv={self.model.nv}."
            )

        self.dof = len(self.joint_names)

        self.joint_ids = np.zeros(
            self.dof,
            dtype=int,
        )

        self.qpos_addresses = np.zeros(
            self.dof,
            dtype=int,
        )

        self.dof_addresses = np.zeros(
            self.dof,
            dtype=int,
        )

        self.joint_limited = np.zeros(
            self.dof,
            dtype=bool,
        )

        self.joint_limits = np.zeros(
            (self.dof, 2),
            dtype=np.float64,
        )

        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.model.joint(
                joint_name
            ).id

            self.joint_ids[i] = joint_id

            self.qpos_addresses[i] = (
                self.model.jnt_qposadr[joint_id]
            )

            self.dof_addresses[i] = (
                self.model.jnt_dofadr[joint_id]
            )

            self.joint_limited[i] = bool(
                self.model.jnt_limited[joint_id]
            )

            if self.joint_limited[i]:
                self.joint_limits[i] = (
                    self.model.jnt_range[joint_id]
                )
            else:
                self.joint_limits[i] = [
                    -np.inf,
                    np.inf,
                ]

    def get_joint_limits(self):
        """Return joint limits in radians."""

        return self.joint_limits.copy()

    def get_joint_limited(self):
        """Return joint limit flags."""

        return self.joint_limited.copy()

    def get_joint_names(self):
        """Return joint names."""

        return self.joint_names.copy()

    def print_joint_info(self):
        """Print joint information."""

        print("Joint Information")
        print()

        for i, name in enumerate(self.joint_names):
            minimum, maximum = self.joint_limits[i]

            print(
                f"{name}: "
                f"{minimum:.6f} -> "
                f"{maximum:.6f} rad"
            )

    # Joint state

    def set_joint_positions(
        self,
        q,
        check_limits=True,
    ):
        """Set joint positions and update MuJoCo state."""

        q = np.asarray(
            q,
            dtype=np.float64,
        )

        if q.shape != (self.dof,):
            raise ValueError(
                f"Expected q shape ({self.dof},), "
                f"got {q.shape}."
            )

        if check_limits and not self.is_configuration_valid(q):
            raise ValueError(
                "Joint configuration is outside "
                "the joint limits."
            )

        self.data.qpos[
            self.qpos_addresses
        ] = q

        mujoco.mj_forward(
            self.model,
            self.data,
        )

    def get_joint_positions(self):
        """Return current joint positions."""

        return self.data.qpos[
            self.qpos_addresses
        ].copy()

    def is_configuration_valid(self, q):
        """Check whether a configuration is within limits."""

        q = np.asarray(
            q,
            dtype=np.float64,
        )

        if q.shape != (self.dof,):
            return False

        lower = self.joint_limits[:, 0]
        upper = self.joint_limits[:, 1]

        return bool(
            np.all(q >= lower)
            and np.all(q <= upper)
        )

    # Frame transformations

    def world_to_base(self, position_world):
        """Transform a world position into MuJoCo base frame."""

        position_world = np.asarray(
            position_world,
            dtype=np.float64,
        )

        base_id = self.body_ids["base_link"]

        base_position = self.data.xpos[
            base_id
        ]

        base_rotation = self.data.xmat[
            base_id
        ].reshape(3, 3)

        return (
            base_rotation.T
            @ (position_world - base_position)
        )

    def base_to_world(self, position_base):
        """Transform a MuJoCo base position into world frame."""

        position_base = np.asarray(
            position_base,
            dtype=np.float64,
        )

        base_id = self.body_ids["base_link"]

        base_position = self.data.xpos[
            base_id
        ]

        base_rotation = self.data.xmat[
            base_id
        ].reshape(3, 3)

        return (
            base_rotation
            @ position_base
            + base_position
        )

    # Forward kinematics

    def forward_kinematics(self, q=None):
        """
        Calculate forward kinematics in MuJoCo base frame.

        Returns:
            dict containing EoF and body positions.
        """

        if q is not None:
            self.set_joint_positions(q)

        joint_positions = np.zeros(
            (len(self.body_names), 3),
            dtype=np.float64,
        )

        for i, body_name in enumerate(
            self.body_names
        ):
            body_id = self.body_ids[
                body_name
            ]

            position_world = self.data.xpos[
                body_id
            ].copy()

            joint_positions[i] = (
                self.world_to_base(
                    position_world
                )
            )

        eof = joint_positions[
            self.body_names.index(
                "end_effector"
            )
        ].copy()

        return {
            "eof": eof,
            "joint_positions": joint_positions,
            "body_names": self.body_names.copy(),
        }

    # Body positions

    def get_body_position(
        self,
        body_name,
        frame="base",
    ):
        """Get body position in the requested frame."""

        if body_name not in self.body_ids:
            raise ValueError(
                f"Unknown body: {body_name}"
            )

        body_id = self.body_ids[
            body_name
        ]

        position_world = self.data.xpos[
            body_id
        ].copy()

        if frame == "world":
            return position_world

        if frame == "base":
            return self.world_to_base(
                position_world
            )

        raise ValueError(
            f"Unsupported frame: {frame}"
        )

    def get_eof_position(self, frame="base"):
        """Get end-effector position."""

        return self.get_body_position(
            "end_effector",
            frame=frame,
        )

    # Jacobian

    def _get_jacobian(
        self,
        body_name,
        angular=False,
        frame="base",
    ):
        """Calculate a body Jacobian."""

        if body_name not in self.body_ids:
            raise ValueError(
                f"Unknown body: {body_name}"
            )

        if frame not in ("world", "base"):
            raise ValueError(
                f"Unsupported frame: {frame}"
            )

        body_id = self.body_ids[
            body_name
        ]

        jacobian_position = np.zeros(
            (3, self.model.nv),
            dtype=np.float64,
        )

        jacobian_rotation = np.zeros(
            (3, self.model.nv),
            dtype=np.float64,
        )

        mujoco.mj_jacBody(
            self.model,
            self.data,
            jacobian_position,
            jacobian_rotation,
            body_id,
        )

        if angular:
            jacobian = jacobian_rotation
        else:
            jacobian = jacobian_position

        if frame == "base":
            base_id = self.body_ids[
                "base_link"
            ]

            base_rotation = self.data.xmat[
                base_id
            ].reshape(3, 3)

            jacobian = (
                base_rotation.T
                @ jacobian
            )

        return jacobian[
            :,
            self.dof_addresses,
        ]

    def get_position_jacobian(
        self,
        body_name="end_effector",
        frame="base",
    ):
        """Return the linear position Jacobian."""

        return self._get_jacobian(
            body_name,
            angular=False,
            frame=frame,
        )

    def get_angular_jacobian(
        self,
        body_name="end_effector",
        frame="base",
    ):
        """Return the angular Jacobian."""

        return self._get_jacobian(
            body_name,
            angular=True,
            frame=frame,
        )

    # Actuator information

    def _initialize_actuator_info(self):
        self.actuator_names = [
            self.model.actuator(
                i
            ).name
            for i in range(
                self.model.nu
            )
        ]

        self.actuator_joint_indices = []

        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[
                i,
                0,
            ]

            if joint_id >= 0:
                joint_index = self.joint_names.index(
                    self.model.joint(
                        joint_id
                    ).name
                )
                self.actuator_joint_indices.append(
                    joint_index
                )
            else:
                self.actuator_joint_indices.append(
                    -1
                )

    @property
    def actuator_count(self):
        """Return number of actuators."""

        return self.model.nu

    def get_actuator_joint_indices(self):
        """Return joint index for each actuator."""

        return np.asarray(
            self.actuator_joint_indices,
            dtype=int,
        )


if __name__ == "__main__":
    from pathlib import Path
    import sys

    script_dir = Path(
        __file__
    ).resolve().parent

    package_dir = script_dir.parent.parent

    config_dir = package_dir / "config"

    sys.path.insert(
        0,
        str(config_dir.parent / "scripts"),
    )

    from config.robot_config import RobotConfig

    config_path = (
        config_dir / "robot.yaml"
    )

    config = RobotConfig(
        config_path
    )

    kinematics = RobotKinematics(
        config.get_model_path()
    )

    kinematics.print_joint_info()

    print()
    print("Joint limits:")
    print(
        kinematics.get_joint_limits()
    )

    print()
    print("Current EoF:")
    print(
        kinematics.get_eof_position()
    )