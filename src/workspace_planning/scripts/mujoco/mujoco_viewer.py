#!/usr/bin/env python3

from pathlib import Path
import sys

import mujoco
import mujoco.viewer
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = SCRIPTS_DIR.parent

sys.path.insert(
    0,
    str(SCRIPTS_DIR),
)

from config.robot_config import RobotConfig
from kinematics.robot_kinematics import RobotKinematics
from workspace.frame_transform import FrameTransform


CONFIG_PATH = (
    PACKAGE_DIR
    / "config"
    / "robot.yaml"
)


JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
]


def get_joint_id(model, joint_name):
    """Get joint ID."""

    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    if joint_id < 0:
        raise ValueError(
            f"Joint not found: {joint_name}"
        )

    return joint_id


def get_joint_qpos_address(
    model,
    joint_name,
):
    """Get qpos address for a joint."""

    joint_id = get_joint_id(
        model,
        joint_name,
    )

    return model.jnt_qposadr[
        joint_id
    ]


def get_joint_limits(
    model,
    joint_name,
):
    """Get joint limits from XML."""

    joint_id = get_joint_id(
        model,
        joint_name,
    )

    return model.jnt_range[
        joint_id
    ].copy()


def get_joint_positions(
    model,
    data,
):
    """Get current joint positions."""

    q = np.zeros(
        len(JOINT_NAMES),
        dtype=np.float64,
    )

    for index, joint_name in enumerate(
        JOINT_NAMES
    ):
        qpos_address = (
            get_joint_qpos_address(
                model,
                joint_name,
            )
        )

        q[index] = data.qpos[
            qpos_address
        ]

    return q


def map_control_to_joint(
    model,
    data,
):
    """Map control sliders to joint positions."""

    count = min(
        model.nu,
        len(JOINT_NAMES),
    )

    changed = False

    for index in range(count):
        joint_name = JOINT_NAMES[index]

        joint_id = get_joint_id(
            model,
            joint_name,
        )

        qpos_address = (
            model.jnt_qposadr[
                joint_id
            ]
        )

        minimum, maximum = (
            model.jnt_range[
                joint_id
            ]
        )

        control_value = data.ctrl[
            index
        ]

        new_position = np.clip(
            control_value,
            minimum,
            maximum,
        )

        old_position = data.qpos[
            qpos_address
        ]

        if not np.isclose(
            old_position,
            new_position,
            atol=1e-8,
        ):
            data.qpos[
                qpos_address
            ] = new_position

            changed = True

    return changed


def reset_robot(
    model,
    data,
):
    """Reset robot to XML initial pose."""

    data.qpos[:] = model.qpos0[:]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )


def print_model_info(
    model,
    model_path,
):
    """Print model information."""

    print("=" * 50)
    print("MuJoCo Robot Viewer + FK")
    print("=" * 50)

    print(
        f"Model : {model_path}"
    )

    print(
        f"DOF   : {model.nq}"
    )

    print()
    print("Joint Limits")
    print("-" * 50)

    for joint_name in JOINT_NAMES:
        minimum, maximum = (
            get_joint_limits(
                model,
                joint_name,
            )
        )

        print(
            f"{joint_name:<8}: "
            f"{np.degrees(minimum):>8.2f} "
            f"to "
            f"{np.degrees(maximum):>8.2f} deg"
        )

    print()
    print("Actuators")
    print("-" * 50)

    for actuator_index in range(
        model.nu
    ):
        actuator_name = (
            model.actuator(
                actuator_index
            ).name
        )

        joint_id = (
            model.actuator_trnid[
                actuator_index,
                0,
            ]
        )

        if joint_id >= 0:
            joint_name = (
                model.joint(
                    joint_id
                ).name
            )
        else:
            joint_name = "none"

        print(
            f"{actuator_index}: "
            f"{actuator_name:<10} "
            f"-> {joint_name}"
        )


def print_frame_info():
    """Print coordinate frame information."""

    print()
    print("Coordinate Frames")
    print("-" * 50)

    print(
        "Source frame : mujoco_base"
    )

    print(
        "Target frame : base_link"
    )

    print()
    print(
        "Transform is provided by "
        "FrameTransform."
    )


def print_fk(
    kinematics,
    model,
    data,
):
    """Print FK and transformed positions."""

    q = get_joint_positions(
        model,
        data,
    )

    result = (
        kinematics.forward_kinematics(
            q
        )
    )

    eof_mujoco = result[
        "eof"
    ]

    joint_positions_mujoco = (
        result[
            "joint_positions"
        ]
    )

    eof_robot = (
        FrameTransform
        .mujoco_base_to_robot_base(
            eof_mujoco
        )
    )

    joint_positions_robot = (
        FrameTransform
        .transform_positions(
            joint_positions_mujoco
        )
    )

    print()
    print("=" * 50)
    print("Forward Kinematics")
    print("=" * 50)

    print()
    print("Joint Configuration")
    print("-" * 50)

    for index, joint_name in enumerate(
        JOINT_NAMES
    ):
        print(
            f"{joint_name:<8}: "
            f"{np.degrees(q[index]):>8.2f} deg "
            f"({q[index]:>8.4f} rad)"
        )

    print()
    print("End-Effector")
    print("-" * 50)

    print(
        "mujoco_base:"
    )

    print(
        f"  X = {eof_mujoco[0]: .4f} m"
    )

    print(
        f"  Y = {eof_mujoco[1]: .4f} m"
    )

    print(
        f"  Z = {eof_mujoco[2]: .4f} m"
    )

    print()

    print(
        "base_link:"
    )

    print(
        f"  X = {eof_robot[0]: .4f} m"
    )

    print(
        f"  Y = {eof_robot[1]: .4f} m"
    )

    print(
        f"  Z = {eof_robot[2]: .4f} m"
    )

    print()
    print("Body Positions - base_link")
    print("-" * 50)

    body_names = result[
        "body_names"
    ]

    for index, body_name in enumerate(
        body_names
    ):
        position = (
            joint_positions_robot[
                index
            ]
        )

        print(
            f"{body_name:<15}: "
            f"X={position[0]:>7.3f} "
            f"Y={position[1]:>7.3f} "
            f"Z={position[2]:>7.3f}"
        )


def main():
    config = RobotConfig(
        CONFIG_PATH
    )

    model_path = (
        config.get_model_path()
    )

    if not model_path.exists():
        raise FileNotFoundError(
            f"MuJoCo model not found:\n"
            f"{model_path}"
        )

    model = (
        mujoco.MjModel.from_xml_path(
            str(model_path)
        )
    )

    data = mujoco.MjData(
        model
    )

    kinematics = RobotKinematics(
        model_path
    )

    print_model_info(
        model,
        model_path,
    )

    print_frame_info()

    print()
    print("Control")
    print("-" * 50)

    for index, joint_name in enumerate(
        JOINT_NAMES
    ):
        print(
            f"motor{index + 1} -> "
            f"{joint_name}"
        )

    print()
    print(
        "Use the MuJoCo control sliders "
        "to move the joints."
    )

    reset_robot(
        model,
        data,
    )

    last_q = (
        get_joint_positions(
            model,
            data,
        )
    )

    last_ctrl = (
        data.ctrl.copy()
    )

    print_fk(
        kinematics,
        model,
        data,
    )

    with mujoco.viewer.launch_passive(
        model,
        data,
    ) as viewer:

        while viewer.is_running():

            control_changed = (
                not np.allclose(
                    data.ctrl,
                    last_ctrl,
                    atol=1e-8,
                )
            )

            if control_changed:
                map_control_to_joint(
                    model,
                    data,
                )

                mujoco.mj_forward(
                    model,
                    data,
                )

                current_q = (
                    get_joint_positions(
                        model,
                        data,
                    )
                )

                if not np.allclose(
                    current_q,
                    last_q,
                    atol=1e-8,
                ):
                    print_fk(
                        kinematics,
                        model,
                        data,
                    )

                    last_q = (
                        current_q.copy()
                    )

                last_ctrl = (
                    data.ctrl.copy()
                )

            viewer.sync()

    print()
    print("Viewer closed.")


if __name__ == "__main__":
    main()