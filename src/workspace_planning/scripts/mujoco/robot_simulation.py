#!/usr/bin/env python3

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy

from geometry_msgs.msg import Point
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node


SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent

sys.path.insert(
    0,
    str(SCRIPTS_DIR),
)

from config.robot_config import RobotConfig
from kinematics.robot_kinematics import RobotKinematics
from workspace.frame_transform import FrameTransform


def dpinv(A, lam):
    if lam > 0.0:
        return (
            A.T
            @ np.linalg.inv(
                A @ A.T
                + lam ** 2
                * np.eye(A.shape[0])
            )
        )

    return np.linalg.pinv(A)


class RobotSimulation(Node):
    def __init__(self, model_path):
        super().__init__("robot_simulation")

        self.kinematics = RobotKinematics(
            model_path
        )

        self.model = self.kinematics.model
        self.data = self.kinematics.data

        if (
            self.kinematics.actuator_count
            != self.kinematics.dof
        ):
            raise RuntimeError(
                "Number of actuators must match robot DOF."
            )

        self.dt = self.model.opt.timestep

        self.ee_id = (
            self.kinematics.body_ids[
                "end_effector"
            ]
        )

        self.joint_limits = (
            self.kinematics.get_joint_limits()
        )

        self.q_lower = (
            self.joint_limits[:, 0]
        )

        self.q_upper = (
            self.joint_limits[:, 1]
        )

        self.target_mocap_id = (
            self._get_target_mocap_id()
        )

        mujoco.mj_forward(
            self.model,
            self.data,
        )

        target_world = (
            self.data.mocap_pos[
                self.target_mocap_id
            ].copy()
        )

        target_mujoco_base = (
            self.kinematics.world_to_base(
                target_world
            )
        )

        self.target_position_robot = (
            FrameTransform
            .mujoco_base_to_robot_base(
                target_mujoco_base
            )
        )

        self.initial_target_robot = (
            self.target_position_robot.copy()
        )

        self.running = True

        self.q_des = (
            self.kinematics
            .get_joint_positions()
        )

        self.actuator_indices = (
            self.kinematics
            .get_actuator_joint_indices()
        )

        self.path = None
        self.path_index = 0
        self.path_active = False
        self.current_waypoint = None

        self.waypoint_tolerance = 0.015

        self.target_subscriber = (
            self.create_subscription(
                Point,
                "/planning/target_position",
                self.target_callback,
                10,
            )
        )

        self.path_subscriber = (
            self.create_subscription(
                PathMessage,
                "/planning/a_star_path",
                self.path_callback,
                10,
            )
        )

        self.eof_publisher = (
            self.create_publisher(
                Point,
                "/planning/current_eof",
                10,
            )
        )

        self.target_publisher = (
            self.create_publisher(
                Point,
                "/planning/current_target",
                10,
            )
        )

        self.get_logger().info(
            "Robot simulation started."
        )

    def _get_target_mocap_id(self):
        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "target",
        )

        if body_id < 0:
            raise RuntimeError(
                'Body "target" was not found.'
            )

        mocap_id = (
            self.model.body_mocapid[
                body_id
            ]
        )

        if mocap_id < 0:
            raise RuntimeError(
                'Body "target" must use mocap="true".'
            )

        return int(mocap_id)

    def target_callback(self, message):
        target = np.array(
            [
                message.x,
                message.y,
                message.z,
            ],
            dtype=np.float64,
        )

        try:
            self.set_target_position_robot(
                target
            )

            self.get_logger().info(
                "Target base_link: "
                f"{np.round(target, 3).tolist()}"
            )

        except ValueError as error:
            self.get_logger().error(
                str(error)
            )

    def path_callback(self, message):
        if len(message.poses) == 0:
            self.get_logger().warning(
                "Received empty A* path."
            )

            self.path = None
            self.path_index = 0
            self.current_waypoint = None
            self.path_active = False

            return

        path_positions = []

        for pose in message.poses:
            position = np.array(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ],
                dtype=np.float64,
            )

            path_positions.append(
                position
            )

        self.path = np.asarray(
            path_positions,
            dtype=np.float64,
        )

        if len(self.path) <= 1:
            self.path_index = 0
            self.current_waypoint = None
            self.path_active = False

            self.get_logger().warning(
                "A* path does not contain "
                "enough points."
            )

            return

        self.path_index = 1

        self.current_waypoint = (
            self.path[self.path_index].copy()
        )

        self.path_active = True

        final_target = (
            self.path[-1].copy()
        )

        self.set_target_position_robot(
            final_target
        )

        path_length = 0.0

        for index in range(
            len(self.path) - 1
        ):
            path_length += np.linalg.norm(
                self.path[index + 1]
                - self.path[index]
            )

        self.get_logger().info(
            "A* path received: "
            f"{len(self.path)} points"
        )

        self.get_logger().info(
            f"A* path length: "
            f"{path_length:.3f} m"
        )

        first_waypoint = np.round(
            self.current_waypoint,
            3,
        ).tolist()

        self.get_logger().info(
            "Path execution started. "
            f"First waypoint: {first_waypoint}"
        )

    def get_target_position_robot(self):
        return (
            self.target_position_robot.copy()
        )

    def set_target_position_robot(
        self,
        position,
    ):
        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if position.shape != (3,):
            raise ValueError(
                "Target position must have shape (3,)."
            )

        self.target_position_robot = (
            position.copy()
        )

    def target_robot_to_world(self):
        target_robot = (
            self.get_target_position_robot()
        )

        target_mujoco_base = (
            FrameTransform
            .robot_base_to_mujoco_base(
                target_robot
            )
        )

        target_world = (
            self.kinematics.base_to_world(
                target_mujoco_base
            )
        )

        return target_world

    def update_target_mujoco(self):
        target_world = (
            self.target_robot_to_world()
        )

        self.data.mocap_pos[
            self.target_mocap_id
        ] = target_world

    def get_current_eof_robot(self):
        tip_mujoco = (
            self.kinematics
            .get_eof_position(
                frame="world"
            )
        )

        tip_robot = (
            FrameTransform
            .mujoco_base_to_robot_base(
                self.kinematics.world_to_base(
                    tip_mujoco
                )
            )
        )

        return tip_robot

    def update_waypoint(self, current_eof):
        if (
            not self.path_active
            or self.path is None
            or self.current_waypoint is None
        ):
            return

        distance = np.linalg.norm(
            self.current_waypoint
            - current_eof
        )

        if distance > self.waypoint_tolerance:
            return

        reached_waypoint = np.round(
            self.current_waypoint,
            3,
        ).tolist()

        self.get_logger().info(
            "Waypoint reached: "
            f"{reached_waypoint}"
        )

        self.path_index += 1

        if self.path_index >= len(
            self.path
        ):
            self.current_waypoint = None
            self.path_active = False

            self.get_logger().info(
                "A* path completed."
            )

            return

        self.current_waypoint = (
            self.path[self.path_index].copy()
        )

        next_waypoint = np.round(
            self.current_waypoint,
            3,
        ).tolist()

        self.get_logger().info(
            "Next waypoint: "
            f"{next_waypoint}"
        )

    def get_control_target(self):
        if (
            self.path_active
            and self.current_waypoint is not None
        ):
            return (
                self.current_waypoint.copy()
            )

        return (
            self.get_target_position_robot()
        )

    def publish_current_eof(
        self,
        position,
    ):
        message = Point()

        message.x = float(position[0])
        message.y = float(position[1])
        message.z = float(position[2])

        self.eof_publisher.publish(
            message
        )

    def publish_current_target(self):
        target = (
            self.get_target_position_robot()
        )

        message = Point()

        message.x = float(target[0])
        message.y = float(target[1])
        message.z = float(target[2])

        self.target_publisher.publish(
            message
        )

    def run(
        self,
        gain=3.0,
        damping=0.05,
        vmax=0.4,
        dqmax=1.5,
    ):
        try:
            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
            ) as viewer:

                step_count = 0

                while (
                    viewer.is_running()
                    and self.running
                    and rclpy.ok()
                ):
                    rclpy.spin_once(
                        self,
                        timeout_sec=0.0,
                    )

                    self.update_target_mujoco()

                    mujoco.mj_forward(
                        self.model,
                        self.data,
                    )

                    tip_mujoco = (
                        self.kinematics
                        .get_eof_position(
                            frame="world"
                        )
                    )

                    tip_robot = (
                        FrameTransform
                        .mujoco_base_to_robot_base(
                            self.kinematics.world_to_base(
                                tip_mujoco
                            )
                        )
                    )

                    self.update_waypoint(
                        tip_robot
                    )

                    control_target = (
                        self.get_control_target()
                    )

                    target_mujoco_base = (
                        FrameTransform
                        .robot_base_to_mujoco_base(
                            control_target
                        )
                    )

                    control_target_world = (
                        self.kinematics
                        .base_to_world(
                            target_mujoco_base
                        )
                    )

                    error = (
                        control_target_world
                        - tip_mujoco
                    )

                    velocity = (
                        gain * error
                    )

                    velocity_norm = (
                        np.linalg.norm(
                            velocity
                        )
                    )

                    if velocity_norm > vmax:
                        velocity *= (
                            vmax
                            / velocity_norm
                        )

                    J = (
                        self.kinematics
                        .get_position_jacobian(
                            "end_effector",
                            frame="world",
                        )
                    )

                    J_pinv = dpinv(
                        J,
                        damping,
                    )

                    dq = (
                        J_pinv
                        @ velocity
                    )

                    max_dq = np.max(
                        np.abs(dq)
                    )

                    if max_dq > dqmax:
                        dq *= (
                            dqmax
                            / max_dq
                        )

                    self.q_des += (
                        dq * self.dt
                    )

                    q_now = (
                        self.kinematics
                        .get_joint_positions()
                    )

                    self.q_des = np.clip(
                        self.q_des,
                        q_now - 0.3,
                        q_now + 0.3,
                    )

                    self.q_des = np.clip(
                        self.q_des,
                        self.q_lower,
                        self.q_upper,
                    )

                    self.data.ctrl[:] = (
                        self.q_des[
                            self.actuator_indices
                        ]
                    )

                    mujoco.mj_step(
                        self.model,
                        self.data,
                    )

                    self.publish_current_eof(
                        tip_robot
                    )

                    self.publish_current_target()

                    step_count += 1

                    if step_count % 5 == 0:
                        viewer.sync()

                    if step_count % 500 == 0:
                        error_mm = (
                            np.linalg.norm(
                                error
                            )
                            * 1000.0
                        )

                        eof_robot = np.round(
                            tip_robot,
                            3,
                        ).tolist()

                        print(
                            f"EoF base_link: "
                            f"{eof_robot}"
                        )

                        if (
                            self.current_waypoint
                            is not None
                        ):
                            waypoint = np.round(
                                self.current_waypoint,
                                3,
                            ).tolist()

                            print(
                                f"Waypoint: "
                                f"{waypoint}"
                            )

                        target_robot = np.round(
                            self.target_position_robot,
                            3,
                        ).tolist()

                        print(
                            f"Target base_link: "
                            f"{target_robot}"
                        )

                        print(
                            f"Position error: "
                            f"{error_mm:.1f} mm"
                        )

                    time.sleep(
                        self.dt
                    )

        finally:
            self.running = False


def main():
    rclpy.init()

    package_dir = (
        Path(__file__)
        .resolve()
        .parent.parent.parent
    )

    config_path = (
        package_dir
        / "config"
        / "robot.yaml"
    )

    config = RobotConfig(
        config_path
    )

    model_path = (
        config.get_model_path()
    )

    simulation = RobotSimulation(
        model_path
    )

    try:
        simulation.run()

    finally:
        simulation.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()