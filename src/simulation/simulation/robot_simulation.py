#!/usr/bin/env python3

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from simulation.frame_transform import FrameTransform


class RobotSimulation(Node):
    def __init__(self):
        super().__init__("robot_simulation")

        share = Path(get_package_share_directory("cra"))

        with (share / "config/robot.yaml").open(
            encoding="utf-8"
        ) as file:
            config = yaml.safe_load(file)

        robot = config["robot"]

        model_path = share / robot["model"]["path"]

        self.model = mujoco.MjModel.from_xml_path(
            str(model_path)
        )
        self.data = mujoco.MjData(self.model)

        mujoco.mj_forward(self.model, self.data)

        self.dt = float(self.model.opt.timestep)
        self.joint_names = list(robot["joints"])

        if len(self.joint_names) != 5:
            raise ValueError("CRA must have five joints")

        joint_ids = [
            self.model.joint(name).id
            for name in self.joint_names
        ]

        self.qpos_ids = [
            int(self.model.jnt_qposadr[joint_id])
            for joint_id in joint_ids
        ]

        self.qvel_ids = [
            int(self.model.jnt_dofadr[joint_id])
            for joint_id in joint_ids
        ]

        self.actuator_ids = []

        for joint_id in joint_ids:
            actuators = [
                i
                for i in range(self.model.nu)
                if (
                    self.model.actuator_trntype[i]
                    == mujoco.mjtTrn.mjTRN_JOINT
                    and self.model.actuator_trnid[i, 0] == joint_id
                )
            ]

            if len(actuators) != 1:
                raise ValueError(
                    f"Invalid actuator for joint {joint_id}"
                )

            self.actuator_ids.append(actuators[0])

        self.q_lower = self.model.jnt_range[
            joint_ids, 0
        ].copy()

        self.q_upper = self.model.jnt_range[
            joint_ids, 1
        ].copy()

        frames = robot["frames"]

        self.base_id = self.model.body(
            frames["base"]
        ).id

        self.eof_id = self.model.body(
            frames["end_effector"]
        ).id

        target_body_id = self.model.body("target").id

        self.target_mocap_id = int(
            self.model.body_mocapid[target_body_id]
        )

        self.target_geom_id = self.model.geom("target").id

        if self.target_mocap_id < 0:
            raise ValueError("Target must be a mocap body")

        self.obstacles = {}

        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_id,
            )

            if not name or not name.startswith("obstacle_"):
                continue

            mocap_id = int(
                self.model.body_mocapid[body_id]
            )

            geom_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"{name}_geom",
            )

            if mocap_id < 0 or geom_id < 0:
                raise ValueError(
                    f"Invalid obstacle definition: {name}"
                )

            if (
                self.model.geom_type[geom_id]
                != mujoco.mjtGeom.mjGEOM_BOX
                or self.model.geom_bodyid[geom_id] != body_id
            ):
                raise ValueError(
                    f"{name} must have a box geom"
                )

            self.obstacles[name] = {
                "mocap_id": mocap_id,
                "geom_id": geom_id,
            }

        self.obstacle_names = sorted(self.obstacles)

        # Use the established CRA base-coordinate conversion.
        origin = np.asarray(
            FrameTransform.mujoco_base_to_robot_base(
                np.zeros(3)
            ),
            dtype=float,
        )

        self.base_axes = np.column_stack([
            np.asarray(
                FrameTransform.mujoco_base_to_robot_base(
                    np.eye(3)[axis]
                ),
                dtype=float,
            ) - origin
            for axis in range(3)
        ])

        if (
            not np.allclose(
                self.base_axes.T @ self.base_axes,
                np.eye(3),
                atol=1e-6,
            )
            or not np.isclose(
                np.linalg.det(self.base_axes),
                1.0,
                atol=1e-6,
            )
        ):
            raise ValueError(
                "FrameTransform must define a rigid rotation"
            )

        tracking = config.get(
            "motion", {}
        ).get("tracking", {})

        self.max_velocity = float(
            tracking.get("max_joint_velocity", 0.5)
        )

        self.command_timeout = float(
            tracking.get("command_timeout", 0.2)
        )

        if (
            self.dt <= 0
            or not np.isfinite(self.max_velocity)
            or not np.isfinite(self.command_timeout)
            or self.max_velocity <= 0
            or self.command_timeout <= 0
        ):
            raise ValueError("Invalid simulation control parameters")

        self.q_des = self.joint_positions()
        self.command = np.zeros(5)
        self.last_command = None
        self.last_debug_time = 0.0
        self.holding_position = True

        self.pending_target = None
        self.pending_obstacles = {}

        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.eof_pub = self.create_publisher(
            Point,
            "/planning/current_eof",
            10,
        )

        self.target_pub = self.create_publisher(
            Point,
            "/motion/target_position",
            10,
        )

        self.obstacle_pose_pub = self.create_publisher(
            PoseArray,
            "/simulation/obstacle_poses",
            10,
        )

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.obstacle_marker_pub = self.create_publisher(
            MarkerArray,
            "/simulation/obstacles",
            marker_qos,
        )

        self.target_marker_pub = self.create_publisher(
            Marker,
            "/simulation/target_marker",
            marker_qos,
        )

        self.create_subscription(
            Float64MultiArray,
            "/motion/joint_velocity_command",
            self.velocity_callback,
            10,
        )

        self.create_subscription(
            Point,
            "/simulation/set_target",
            self.target_callback,
            10,
        )

        for name in self.obstacle_names:
            self.create_subscription(
                Point,
                f"/simulation/set_{name}",
                lambda msg, obstacle_name=name:
                self.obstacle_callback(obstacle_name, msg),
                10,
            )

        self.get_logger().info(
            "CRA MuJoCo simulation ready"
        )

        self.get_logger().info(
            f"Obstacle names: {self.obstacle_names}"
        )

    def joint_positions(self):
        return self.data.qpos[self.qpos_ids].copy()

    def base_rotation(self):
        return self.data.xmat[
            self.base_id
        ].reshape(3, 3).copy()

    def world_to_robot_base(self, position):
        position = np.asarray(position, dtype=float)

        mujoco_base = (
            self.base_rotation().T
            @ (
                position
                - self.data.xpos[self.base_id]
            )
        )

        return np.asarray(
            FrameTransform.mujoco_base_to_robot_base(
                mujoco_base
            ),
            dtype=float,
        )

    def robot_base_to_world(self, position):
        mujoco_base = np.asarray(
            FrameTransform.robot_base_to_mujoco_base(
                np.asarray(position, dtype=float)
            ),
            dtype=float,
        )

        return (
            self.data.xpos[self.base_id]
            + self.base_rotation() @ mujoco_base
        )

    @staticmethod
    def point_array(message):
        return np.array(
            [message.x, message.y, message.z],
            dtype=float,
        )

    def velocity_callback(self, message):
        velocity = np.asarray(message.data, dtype=float)

        if velocity.shape != (5,) or not np.isfinite(velocity).all():
            self.get_logger().error("Invalid joint velocity command")
            self.command.fill(0.0)
            self.last_command = None
            if not self.holding_position:
                self.q_des = self.joint_positions()
            self.holding_position = True
            return

        self.command = np.clip(
            velocity,
            -self.max_velocity,
            self.max_velocity,
        )

        moving = bool(np.any(self.command))
        if not moving and not self.holding_position:
            # Capture the hold position once, not on every zero command.
            self.q_des = self.joint_positions()

        self.holding_position = not moving
        self.last_command = time.monotonic()

    def target_callback(self, message):
        position = self.point_array(message)

        if not np.isfinite(position).all():
            self.get_logger().error(
                "Invalid target position"
            )
            return

        self.pending_target = position

    def obstacle_callback(self, name, message):
        position = self.point_array(message)

        if not np.isfinite(position).all():
            self.get_logger().error(
                f"Invalid obstacle position: {name}"
            )
            return

        self.pending_obstacles[name] = position

    def update_control(self):
        if self.pending_target is not None:
            self.data.mocap_pos[
                self.target_mocap_id
            ] = self.robot_base_to_world(
                self.pending_target
            )

            self.get_logger().info(
                "Target moved to base_link: "
                f"{self.pending_target.tolist()}"
            )

            self.pending_target = None

        for name, position in self.pending_obstacles.items():
            mocap_id = self.obstacles[name]["mocap_id"]

            self.data.mocap_pos[mocap_id] = (
                self.robot_base_to_world(position)
            )

            self.get_logger().info(
                f"{name} moved to base_link: "
                f"{position.tolist()}"
            )

        self.pending_obstacles.clear()

        if self.last_command is not None:
            if time.monotonic() - self.last_command <= self.command_timeout:
                if not self.holding_position:
                    self.q_des += self.command * self.dt
            else:
                self.command.fill(0.0)
                self.last_command = None
                if not self.holding_position:
                    self.q_des = self.joint_positions()
                self.holding_position = True

        current = self.joint_positions()

        if not self.holding_position:
            self.q_des = np.clip(
                self.q_des,
                current - 0.12,
                current + 0.12,
            )

        self.q_des = np.clip(
            self.q_des,
            self.q_lower,
            self.q_upper,
        )

        self.data.ctrl[self.actuator_ids] = self.q_des

        now = time.monotonic()
        if (
            self.last_command is not None
            and now - self.last_command <= self.command_timeout
            and now - self.last_debug_time >= 1.0
        ):
            self.last_debug_time = now
            actual_velocity = self.data.qvel[self.qvel_ids]
            actuator_force = self.data.actuator_force[self.actuator_ids[0]]

            self.get_logger().info(
                "Joint1: "
                f"cmd={self.command[0]:.3f}, "
                f"actual_vel={actual_velocity[0]:.3f}, "
                f"q_des={self.q_des[0]:.3f}, "
                f"q_actual={current[0]:.3f}, "
                f"force={actuator_force:.3f}, "
                f"contacts={self.data.ncon}"
            )

    @staticmethod
    def rotation_to_quaternion(rotation):
        quaternion = np.empty(4, dtype=float)

        mujoco.mju_mat2Quat(
            quaternion,
            np.asarray(
                rotation, dtype=float
            ).reshape(9),
        )

        return quaternion

    @staticmethod
    def set_pose(pose, position, quaternion):
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])

        # MuJoCo uses [w, x, y, z].
        pose.orientation.w = float(quaternion[0])
        pose.orientation.x = float(quaternion[1])
        pose.orientation.y = float(quaternion[2])
        pose.orientation.z = float(quaternion[3])

    def make_geom_marker(
        self,
        geom_id,
        name,
        marker_id,
        stamp,
    ):
        marker = Marker()

        marker.header.stamp = stamp
        marker.header.frame_id = "base_link"

        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Use the actual geom pose, including any local geom offset.
        world_position = self.data.geom_xpos[
            geom_id
        ].copy()

        world_rotation = self.data.geom_xmat[
            geom_id
        ].reshape(3, 3).copy()

        base_rotation = (
            self.base_axes
            @ self.base_rotation().T
            @ world_rotation
        )

        base_quaternion = self.rotation_to_quaternion(
            base_rotation
        )

        self.set_pose(
            marker.pose,
            self.world_to_robot_base(world_position),
            base_quaternion,
        )

        size = (
            self.model.geom_size[geom_id] * 2.0
        )

        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])

        color = self.model.geom_rgba[geom_id]

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

        return marker

    def publish_visualization(self):
        stamp = self.get_clock().now().to_msg()

        obstacle_poses = PoseArray()
        obstacle_poses.header.stamp = stamp
        obstacle_poses.header.frame_id = "mujoco_world"

        obstacle_markers = MarkerArray()

        for index, name in enumerate(
            self.obstacle_names
        ):
            info = self.obstacles[name]
            mocap_id = info["mocap_id"]
            geom_id = info["geom_id"]

            # Preserve the existing world-frame PoseArray.
            pose = self.make_world_pose(mocap_id)
            obstacle_poses.poses.append(pose)

            marker = self.make_geom_marker(
                geom_id,
                name,
                index,
                stamp,
            )

            # The XML parks inactive obstacles at [100, 100, 100].
            # Keep distant obstacles out of the RViz working view.
            position = self.data.mocap_pos[mocap_id]

            if np.linalg.norm(position) > 50.0:
                marker.action = Marker.DELETE

            obstacle_markers.markers.append(marker)

        self.obstacle_pose_pub.publish(
            obstacle_poses
        )

        self.obstacle_marker_pub.publish(
            obstacle_markers
        )

        target_marker = self.make_geom_marker(
            self.target_geom_id,
            "target",
            0,
            stamp,
        )

        self.target_marker_pub.publish(
            target_marker
        )

    def make_world_pose(self, mocap_id):
        from geometry_msgs.msg import Pose

        pose = Pose()

        self.set_pose(
            pose,
            self.data.mocap_pos[mocap_id],
            self.data.mocap_quat[mocap_id],
        )

        return pose

    def publish_feedback(self):
        positions = self.joint_positions()
        velocities = self.data.qvel[
            self.qvel_ids
        ].copy()

        joint_msg = JointState()

        joint_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        joint_msg.name = self.joint_names
        joint_msg.position = positions.tolist()
        joint_msg.velocity = velocities.tolist()

        self.joint_pub.publish(joint_msg)

        eof_world = self.data.xpos[
            self.eof_id
        ].copy()

        eof_robot = self.world_to_robot_base(
            eof_world
        )

        self.eof_pub.publish(
            Point(
                x=float(eof_robot[0]),
                y=float(eof_robot[1]),
                z=float(eof_robot[2]),
            )
        )

        # Preserve the existing motion target coordinate convention.
        target_world = self.data.mocap_pos[
            self.target_mocap_id
        ].copy()

        target_mujoco_base = (
            self.base_rotation().T
            @ (
                target_world
                - self.data.xpos[self.base_id]
            )
        )

        self.target_pub.publish(
            Point(
                x=float(target_mujoco_base[0]),
                y=float(target_mujoco_base[1]),
                z=float(target_mujoco_base[2]),
            )
        )

        self.publish_visualization()

    def run(self):
        with mujoco.viewer.launch_passive(
            self.model,
            self.data,
        ) as viewer:
            step = 0

            self.get_logger().info(
                "MuJoCo viewer started"
            )

            while rclpy.ok() and viewer.is_running():
                start = time.monotonic()

                rclpy.spin_once(
                    self,
                    timeout_sec=0.0,
                )

                with viewer.lock():
                    self.update_control()

                    mujoco.mj_step(
                        self.model,
                        self.data,
                    )

                    step += 1

                    if step % 5 == 0:
                        self.publish_feedback()

                if step % 5 == 0:
                    viewer.sync()

                elapsed = time.monotonic() - start

                time.sleep(
                    max(0.0, self.dt - elapsed)
                )


def main():
    rclpy.init()
    node = None

    try:
        node = RobotSimulation()
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()