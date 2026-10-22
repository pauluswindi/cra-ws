import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory


ROBOT_FROM_MUJOCO_BASE = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


def load_configuration():
    cra_share = Path(get_package_share_directory("cra"))
    config_path = cra_share / "config" / "robot.yaml"

    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    model_path = cra_share / config["robot"]["model"]["path"]

    if not model_path.is_file():
        raise FileNotFoundError(
            f"MuJoCo model not found: {model_path}"
        )

    return config, model_path


def generate_workspace(config, model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    joint_names = list(config["robot"]["joints"])
    base_name = config["robot"]["frames"]["base"]
    eof_name = config["robot"]["frames"]["end_effector"]

    joint_ids = []
    qpos_addresses = []
    joint_limits = []

    for name in joint_names:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )

        if joint_id < 0:
            raise ValueError(f"Joint not found: {name}")

        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"Expected hinge joint: {name}")

        if not model.jnt_limited[joint_id]:
            raise ValueError(
                f"Workspace sampling requires joint limits: {name}"
            )

        joint_ids.append(joint_id)
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        joint_limits.append(model.jnt_range[joint_id].copy())

    joint_limits = np.asarray(joint_limits, dtype=np.float64)

    base_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, base_name
    )
    eof_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, eof_name
    )

    if base_id < 0 or eof_id < 0:
        raise ValueError("Base or end-effector body not found")

    body_names = []
    body_ids = []

    for body_id in range(model.nbody):
        name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, body_id
        )

        if name and (
            name.startswith("link") or name == eof_name
        ):
            body_names.append(name)
            body_ids.append(body_id)

    if eof_id not in body_ids:
        body_names.append(eof_name)
        body_ids.append(eof_id)

    num_samples = int(config["workspace"]["num_samples"])
    seed = int(config["workspace"]["random_seed"])

    if num_samples < 2:
        raise ValueError("workspace.num_samples must be at least 2")

    rng = np.random.default_rng(seed)

    q_samples = rng.uniform(
        joint_limits[:, 0],
        joint_limits[:, 1],
        size=(num_samples, len(joint_names)),
    )

    eof_samples = np.empty(
        (num_samples, 3),
        dtype=np.float32,
    )

    joint_position_samples = np.empty(
        (num_samples, len(body_ids), 3),
        dtype=np.float32,
    )

    print(f"Robot       : {config['robot']['name']}")
    print(f"DOF         : {len(joint_names)}")
    print(f"Samples     : {num_samples}")
    print(f"Model       : {model_path}")
    print(f"Frame       : {base_name}")

    for index, joints in enumerate(q_samples):
        data.qpos[qpos_addresses] = joints

        mujoco.mj_forward(model, data)

        base_position = data.xpos[base_id].copy()
        base_rotation = data.xmat[base_id].reshape(3, 3).copy()

        eof_local = (
            base_rotation.T @ (data.xpos[eof_id] - base_position)
        )

        body_world = data.xpos[body_ids]
        body_local = (
            (body_world - base_position) @ base_rotation
        )

        eof_samples[index] = (
            ROBOT_FROM_MUJOCO_BASE @ eof_local
        )

        joint_position_samples[index] = (
            body_local @ ROBOT_FROM_MUJOCO_BASE.T
        )

        if (index + 1) % 1000 == 0:
            print(
                f"Generated {index + 1}/{num_samples} samples"
            )

    return {
        "q": q_samples.astype(np.float32),
        "eof": eof_samples,
        "joint_positions": joint_position_samples,
        "body_names": np.asarray(body_names),
        "joint_names": np.asarray(joint_names),
        "joint_limits": joint_limits,
        "coordinate_frame": np.asarray(base_name),
        "source_coordinate_frame": np.asarray("mujoco_base"),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Generate CRA robot workspace"
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home()
        / ".ros"
        / "cra_planning"
        / "raw_workspace.npz",
    )

    args = parser.parse_args()

    config, model_path = load_configuration()
    workspace = generate_workspace(config, model_path)

    output_path = args.output.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    np.savez_compressed(output_path, **workspace)

    eof = workspace["eof"]

    print("\nWorkspace generation completed")
    print(f"Output      : {output_path}")
    print(f"Joint data  : {workspace['q'].shape}")
    print(f"EOF data    : {eof.shape}")
    print(
        f"Link data   : {workspace['joint_positions'].shape}"
    )
    print(f"EOF minimum : {eof.min(axis=0)}")
    print(f"EOF maximum : {eof.max(axis=0)}")


if __name__ == "__main__":
    main()