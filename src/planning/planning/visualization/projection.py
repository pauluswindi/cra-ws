"""Shared CRA FK projection. Call only with privately owned MuJoCo data."""

import mujoco
import numpy as np

from simulation.frame_transform import FrameTransform


def project_configuration(model, data, qpos_ids, base_id, eof_id, joints):
    joints = np.asarray(joints, dtype=float)
    if joints.shape != (5,) or not np.isfinite(joints).all():
        raise ValueError('Invalid joint configuration')
    data.qpos[qpos_ids] = joints
    mujoco.mj_forward(model, data)
    rotation = data.xmat[base_id].reshape(3, 3)
    local = rotation.T @ (data.xpos[eof_id] - data.xpos[base_id])
    return FrameTransform.mujoco_base_to_robot_base(local)
