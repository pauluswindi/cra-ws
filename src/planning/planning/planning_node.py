#!/usr/bin/env python3

from pathlib import Path

import mujoco
import numpy as np
import rclpy
import yaml

from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from planning.path_planner import PathPlanner
from planning.planning_manager import PlanningManager
from planning.visualization.planning_visualizer import PlanningVisualizer
from simulation.frame_transform import FrameTransform


class PlanningNode(Node):
    def __init__(self):
        super().__init__("planning_node")

        defaults = {
            "graph_path": "",
            "workspace_path": "",
            "model_path": "",
            "config_path": "",
            "feedback_timeout": 1.0,
            "obstacle_timeout": 1.0,
            "goal_tolerance": 0.12,
            "target_clearance": 0.04,
            "publish_filtered_joint_graph": False,
            "execute_path": True,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        paths = {
            name: Path(get(name)).expanduser()
            for name in (
                "graph_path",
                "workspace_path",
                "model_path",
                "config_path",
            )
        }

        for path in paths.values():
            if not path.is_file():
                raise FileNotFoundError(str(path))

        with paths["config_path"].open(encoding="utf-8") as file:
            config = yaml.safe_load(file)

        robot = config["robot"]

        self.joint_names = list(robot["joints"])
        self.base_frame = robot["frames"]["base"]

        if len(self.joint_names) != 5:
            raise ValueError("Expected five robot joints")

        self.show_joint_graph = bool(
            get("publish_filtered_joint_graph")
        )

        self.planner = PathPlanner(
            paths["graph_path"],
            paths["model_path"],
            paths["config_path"],
        )

        self.manager = PlanningManager(
            path_planner=self.planner,
            goal_tolerance=get("goal_tolerance"),
            target_clearance=get("target_clearance"),
            feedback_timeout=get("feedback_timeout"),
            obstacle_timeout=get("obstacle_timeout"),
            execute_path=bool(get("execute_path")),
        )

        self.visualizer = PlanningVisualizer(
            node=self,
            workspace_path=paths["workspace_path"],
            graph_path=paths["graph_path"],
            model_path=paths["model_path"],
            joint_names=self.joint_names,
            base_body=self.base_frame,
            end_effector_body=robot["frames"]["end_effector"],
            frame_id=self.base_frame,
        )

        self.initialize_environment()

        self.status = None
        self.displayed_path = None
        self.graph_update_required = self.show_joint_graph

        durable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_pub = self.create_publisher(
            String,
            "/planning/status",
            durable,
        )

        self.preview_pub = self.create_publisher(
            JointTrajectory,
            "/planning/joint_path_preview",
            durable,
        )

        self.motion_pub = self.create_publisher(
            JointTrajectory,
            "/motion/joint_path",
            10,
        )

        self.stop_pub = self.create_publisher(
            Bool,
            "/motion/stop_requested",
            10,
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10,
        )

        self.obstacle_sub = self.create_subscription(
            PoseArray,
            "/simulation/obstacle_poses",
            self.obstacle_callback,
            10,
        )

        self.target_sub = self.create_subscription(
            Marker,
            "/simulation/target_marker",
            self.target_callback,
            10,
        )

        self.failure_reason_sub = self.create_subscription(
            String,
            "/motion/failure_reason",
            self.failure_reason_callback,
            durable,
        )

        self.motion_status_sub = self.create_subscription(
            String,
            "/motion/execution_status",
            self.motion_status_callback,
            durable,
        )

        self.timer = self.create_timer(
            0.2,
            self.planning_cycle,
        )

        self.set_status("WAITING_FOR_FEEDBACK")

        self.get_logger().info(
            "PlanningNode initialized; "
            f"execute_path={self.manager.execute_path}"
        )

    def initialize_environment(self):
        checker = self.planner.collision_filter

        self.obstacle_names = sorted(
            checker.model.body(i).name
            for i in range(checker.model.nbody)
            if checker.model.body(i).name.startswith("obstacle_")
        )

        target_id = mujoco.mj_name2id(
            checker.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "target",
        )

        if target_id < 0:
            raise ValueError("Target body missing in MuJoCo")

        mujoco.mj_forward(checker.model, checker.data)

        base_id = checker.model.body(self.base_frame).id

        self.base_position_world = (
            checker.data.xpos[base_id].copy()
        )

        self.base_rotation_world = (
            checker.data.xmat[base_id]
            .reshape(3, 3)
            .copy()
        )

    def set_status(self, status):
        if self.status == status:
            return

        self.status = status
        self.status_pub.publish(String(data=status))

        self.get_logger().info(
            f"Planning status: {status}"
        )

    def joint_callback(self, message):
        if len(message.name) != len(message.position):
            return

        if len(set(message.name)) != len(message.name):
            return

        positions = dict(
            zip(message.name, message.position)
        )

        if any(
            name not in positions
            for name in self.joint_names
        ):
            return

        joints = np.asarray(
            [positions[name] for name in self.joint_names],
            dtype=float,
        )

        self.manager.update_joint_state(joints)

    def motion_status_callback(self, message):
        self.manager.update_motion_status(message.data)

    def failure_reason_callback(self, message):
        self.manager.update_failure_reason(message.data)

    def target_callback(self, message):
        if (
            message.action != Marker.ADD
            or message.header.frame_id != self.base_frame
        ):
            return

        target = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )

        if not np.isfinite(target).all():
            self.manager.target_base = None
            self.manager.invalidate_path()
            self.set_status("TARGET_ERROR")
            return

        try:
            mujoco_base = np.asarray(
                FrameTransform.robot_base_to_mujoco_base(
                    target
                ),
                dtype=float,
            )

            world = (
                self.base_position_world
                + self.base_rotation_world @ mujoco_base
            )

            if not np.isfinite(world).all():
                raise ValueError("Invalid target world position")

            changed = self.manager.update_target(
                target,
                world,
            )

            if changed:
                self.clear_visualization()
                self.graph_update_required = (
                    self.show_joint_graph
                )

        except Exception as error:
            self.manager.target_base = None
            self.manager.invalidate_path()
            self.set_status("TARGET_ERROR")
            self.get_logger().error(str(error))

    @staticmethod
    def pose_arrays(pose):
        position = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ],
            dtype=float,
        )

        quaternion = np.array(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ],
            dtype=float,
        )

        norm = np.linalg.norm(quaternion)

        if (
            not np.isfinite(position).all()
            or not np.isfinite(norm)
            or norm < 1e-8
        ):
            raise ValueError("Invalid obstacle pose")

        return position, quaternion / norm

    def obstacle_callback(self, message):
        if (
            message.header.frame_id != "mujoco_world"
            or len(message.poses) != len(self.obstacle_names)
        ):
            self.manager.obstacles_ready = False
            self.manager.invalidate_path()
            self.set_status("OBSTACLE_ERROR")
            return

        try:
            obstacles = []

            for name, pose in zip(
                self.obstacle_names,
                message.poses,
            ):
                position, quaternion = self.pose_arrays(pose)

                removed = np.allclose(
                    position,
                    [100.0, 100.0, 100.0],
                    atol=1e-5,
                    rtol=0.0,
                )

                obstacles.append(
                    (name, position, quaternion, removed)
                )

            changed = self.manager.update_obstacles(
                obstacles
            )

            if changed:
                self.clear_visualization()
                self.graph_update_required = (
                    self.show_joint_graph
                )

        except Exception as error:
            self.manager.obstacles_ready = False
            self.manager.invalidate_path()
            self.set_status("OBSTACLE_ERROR")
            self.get_logger().error(str(error))

    def build_trajectory(self, path):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self.joint_names)

        if path is None:
            return message

        path = np.asarray(path, dtype=float)
        if (
            path.ndim != 2
            or path.shape[1] != len(self.joint_names)
            or len(path) < 2
            or not np.isfinite(path).all()
        ):
            raise ValueError("Invalid joint trajectory path")

        for joints in path:
            point = JointTrajectoryPoint()
            point.positions = joints.tolist()
            message.points.append(point)

        return message

    def publish_preview(self, path):
        self.preview_pub.publish(
            self.build_trajectory(path)
        )

    def clear_visualization(self):
        self.displayed_path = None
        self.publish_preview(None)

        for kind in (
            "initial",
            "invalid",
            "replanned",
        ):
            self.visualizer.clear_path(kind)

    def update_visualization(self):
        path = self.manager.current_path

        if path is None:
            if self.displayed_path is not None:
                self.clear_visualization()
            return

        if (
            self.displayed_path is not None
            and np.array_equal(path, self.displayed_path)
        ):
            return

        self.displayed_path = path.copy()

        self.publish_preview(path)

        self.visualizer.publish_path(
            path,
            "initial",
        )

        self.visualizer.clear_path("invalid")
        self.visualizer.clear_path("replanned")

    def update_graph_visualization(self):
        if not self.graph_update_required:
            return

        self.graph_update_required = False

        if not self.show_joint_graph:
            return

        try:
            graph = self.planner.get_filtered_graph()

            self.visualizer.publish_filtered_graph(graph)

        except Exception as error:
            self.get_logger().error(
                f"Graph visualization failed: {error}"
            )

    def planning_cycle(self):
        decision = self.manager.step()

        self.set_status(decision.status)

        self.update_visualization()
        self.update_graph_visualization()

        if decision.action == "STOP":
            self.stop_pub.publish(
                Bool(data=True)
            )

        elif decision.action == "RELEASE":
            self.stop_pub.publish(
                Bool(data=False)
            )

        elif decision.action == "EXECUTE":
            if decision.path is None:
                self.get_logger().error("Execution decision has no path")
                self.stop_pub.publish(Bool(data=True))
                return

            try:
                trajectory = self.build_trajectory(decision.path)
            except ValueError as error:
                self.get_logger().error(f"Trajectory rejected: {error}")
                self.stop_pub.publish(Bool(data=True))
                return

            self.motion_pub.publish(trajectory)
            self.get_logger().info(
                f"{decision.status}: {len(trajectory.points)} waypoints sent"
            )


def main():
    rclpy.init()

    node = None

    try:
        node = PlanningNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()