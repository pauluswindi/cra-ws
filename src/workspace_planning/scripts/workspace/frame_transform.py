#!/usr/bin/env python3

import numpy as np


class FrameTransform:
    ROBOT_FROM_MUJOCO_BASE = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )

    @classmethod
    def mujoco_base_to_robot_base(
        cls,
        position,
    ):
        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if position.shape != (3,):
            raise ValueError(
                "Position must have shape (3,)."
            )

        return (
            cls.ROBOT_FROM_MUJOCO_BASE
            @ position
        )

    @classmethod
    def transform_positions(
        cls,
        positions,
    ):
        positions = np.asarray(
            positions,
            dtype=np.float64,
        )

        if (
            positions.ndim != 2
            or positions.shape[1] != 3
        ):
            raise ValueError(
                "Positions must have shape (N, 3)."
            )

        return (
            positions
            @ cls.ROBOT_FROM_MUJOCO_BASE.T
        )

    @classmethod
    def mujoco_base_to_robot_vector(
        cls,
        vector,
    ):
        vector = np.asarray(
            vector,
            dtype=np.float64,
        )

        if vector.shape != (3,):
            raise ValueError(
                "Vector must have shape (3,)."
            )

        return (
            cls.ROBOT_FROM_MUJOCO_BASE
            @ vector
        )

    @classmethod
    def robot_base_to_mujoco_base(cls, position):
        position = np.asarray(position, dtype=np.float64)

        if position.shape != (3,):
            raise ValueError("Position must have shape (3,).")

        return cls.ROBOT_FROM_MUJOCO_BASE.T @ position

    @classmethod
    def transform_jacobian(
        cls,
        jacobian,
    ):
        jacobian = np.asarray(
            jacobian,
            dtype=np.float64,
        )

        if (
            jacobian.ndim != 2
            or jacobian.shape[0] != 3
        ):
            raise ValueError(
                "Jacobian must have shape (3, N)."
            )

        return (
            cls.ROBOT_FROM_MUJOCO_BASE
            @ jacobian
        )