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

        if (
            position.shape != (3,)
            or not np.all(np.isfinite(position))
        ):
            raise ValueError(
                "Position must have three finite values"
            )

        return (
            cls.ROBOT_FROM_MUJOCO_BASE
            @ position
        )

    @classmethod
    def robot_base_to_mujoco_base(
        cls,
        position,
    ):
        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if (
            position.shape != (3,)
            or not np.all(np.isfinite(position))
        ):
            raise ValueError(
                "Position must have three finite values"
            )

        return (
            cls.ROBOT_FROM_MUJOCO_BASE.T
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
            or not np.all(np.isfinite(positions))
        ):
            raise ValueError(
                "Positions must have shape (N, 3)"
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
        return cls.mujoco_base_to_robot_base(
            vector
        )

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
            or not np.all(np.isfinite(jacobian))
        ):
            raise ValueError(
                "Jacobian must have shape (3, N)"
            )

        return (
            cls.ROBOT_FROM_MUJOCO_BASE
            @ jacobian
        )