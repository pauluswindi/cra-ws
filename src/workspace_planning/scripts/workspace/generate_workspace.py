#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent.parent

sys.path.insert(
    0,
    str(PACKAGE_DIR / "scripts"),
)

from config.robot_config import RobotConfig
from kinematics.robot_kinematics import RobotKinematics
from workspace.frame_transform import FrameTransform


CONFIG_PATH = (
    PACKAGE_DIR
    / "config"
    / "robot.yaml"
)

OUTPUT_PATH = (
    PACKAGE_DIR
    / "data"
    / "raw_workspace.npz"
)


def generate_workspace(
    kinematics,
    num_samples,
    seed,
):
    """Generate workspace from random joint configurations."""

    rng = np.random.default_rng(seed)

    joint_limits = (
        kinematics.get_joint_limits()
    )

    dof = kinematics.dof

    q_samples = rng.uniform(
        joint_limits[:, 0],
        joint_limits[:, 1],
        size=(num_samples, dof),
    )

    eof_samples = np.zeros(
        (num_samples, 3),
        dtype=np.float32,
    )

    joint_position_samples = np.zeros(
        (
            num_samples,
            len(kinematics.body_names),
            3,
        ),
        dtype=np.float32,
    )

    print("Workspace Generation")
    print(
        f"DOF          : {dof}"
    )
    print(
        f"Samples      : {num_samples}"
    )
    print(
        "Source       : MuJoCo base"
    )
    print(
        "Output       : robot base_link"
    )
    print()

    print("Joint limits:")

    for i, (minimum, maximum) in enumerate(
        joint_limits
    ):
        print(
            f"  joint{i + 1}: "
            f"{minimum:.6f} -> "
            f"{maximum:.6f} rad"
        )

    print()

    for i, q in enumerate(
        q_samples,
        start=1,
    ):
        result = (
            kinematics.forward_kinematics(q)
        )

        eof_mujoco = result["eof"]

        joint_positions_mujoco = (
            result["joint_positions"]
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

        eof_samples[i - 1] = (
            eof_robot
        )

        joint_position_samples[
            i - 1
        ] = joint_positions_robot

        if (
            i % 1000 == 0
            or i == num_samples
        ):
            print(
                f"Generated "
                f"{i:>6}/{num_samples} "
                f"samples"
            )

    return (
        q_samples.astype(
            np.float32
        ),
        eof_samples,
        joint_position_samples,
    )


def save_workspace(
    config,
    kinematics,
    q_samples,
    eof_samples,
    joint_position_samples,
):
    """Save generated workspace data."""

    OUTPUT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    np.savez_compressed(
        OUTPUT_PATH,
        q=q_samples,
        eof=eof_samples,
        joint_positions=(
            joint_position_samples
        ),
        body_names=np.asarray(
            kinematics.body_names
        ),
        joint_names=np.asarray(
            kinematics.joint_names
        ),
        joint_limits=(
            kinematics.get_joint_limits()
        ),
        coordinate_frame=np.asarray(
            config.get_base_frame()
        ),
        source_coordinate_frame=np.asarray(
            "mujoco_base"
        ),
    )


def print_workspace_summary(
    config,
    eof_samples,
):
    """Print workspace bounds."""

    minimum = eof_samples.min(
        axis=0
    )

    maximum = eof_samples.max(
        axis=0
    )

    print()
    print("Workspace Generated")
    print(
        f"Output       : {OUTPUT_PATH}"
    )
    print(
        f"Coordinate   : "
        f"{config.get_base_frame()}"
    )
    print()
    print(
        f"EoF samples  : "
        f"{eof_samples.shape}"
    )
    print()
    print(
        "EoF bounds in base_link:"
    )

    for (
        axis,
        minimum_value,
        maximum_value,
    ) in zip(
        ("X", "Y", "Z"),
        minimum,
        maximum,
    ):
        print(
            f"  {axis}: "
            f"{minimum_value:.4f} "
            f"to "
            f"{maximum_value:.4f} m"
        )


def main():
    print(
        "Loading robot configuration..."
    )
    print(
        f"Config: {CONFIG_PATH}"
    )
    print()

    if not CONFIG_PATH.exists():
        raise FileNotFoundError(
            "Robot configuration not found:\n"
            f"{CONFIG_PATH}"
        )

    config = RobotConfig(
        CONFIG_PATH
    )

    print(
        f"Robot: "
        f"{config.get_robot_name()}"
    )

    print(
        f"Model: "
        f"{config.get_model_path()}"
    )
    print()

    if not config.get_model_path().exists():
        raise FileNotFoundError(
            "MuJoCo model not found:\n"
            f"{config.get_model_path()}"
        )

    kinematics = RobotKinematics(
        config.get_model_path()
    )

    (
        q_samples,
        eof_samples,
        joint_position_samples,
    ) = generate_workspace(
        kinematics,
        config.workspace_num_samples,
        config.workspace_random_seed,
    )

    save_workspace(
        config,
        kinematics,
        q_samples,
        eof_samples,
        joint_position_samples,
    )

    print_workspace_summary(
        config,
        eof_samples,
    )


if __name__ == "__main__":
    main()