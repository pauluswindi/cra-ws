#!/usr/bin/env python3

import time
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
            "goal_index": 100,
            "joint_path_speed": 0.25,
            "feedback_timeout": 1.0,
            "obstacle_timeout": 1.0,
            "publish_filtered_joint_graph": False,
            "goal_tolerance": 0.12,
            "target_clearance": 0.04,
            "execution_segment_delta": 0.5,
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
            robot = yaml.safe_load(file)["robot"]

        self.joint_names = list(robot["joints"])
        self.base_frame = robot["frames"]["base"]

        if len(self.joint_names) != 5:
            raise ValueError("Expected five joints")

        self.planner = PathPlanner(
            paths["graph_path"],
            paths["model_path"],
            paths["config_path"],
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

        with np.load(
            paths["graph_path"],
            allow_pickle=False,
        ) as graph:
            self.joint_eof = np.asarray(
                graph["joint_eof"],
                dtype=float,
            ).copy()

        if self.joint_eof.shape != (
            len(self.planner.joint_q),
            3,
        ):
            raise ValueError(
                "joint_eof and joint_q must have matching rows"
            )

        if not np.isfinite(self.joint_eof).all():
            raise ValueError("Invalid joint_eof")

        self.path_speed = float(get("joint_path_speed"))
        self.feedback_timeout = float(get("feedback_timeout"))
        self.obstacle_timeout = float(get("obstacle_timeout"))
        self.goal_tolerance = float(get("goal_tolerance"))
        self.target_clearance = float(get("target_clearance"))
        self.segment_delta = float(get("execution_segment_delta"))

        self.execute_path = bool(get("execute_path"))
        self.show_joint_graph = bool(
            get("publish_filtered_joint_graph")
        )

        if any(
            not np.isfinite(value) or value <= 0
            for value in (
                self.path_speed,
                self.feedback_timeout,
                self.obstacle_timeout,
                self.goal_tolerance,
                self.segment_delta,
            )
        ):
            raise ValueError("Invalid planning parameters")

        if not 0 <= self.target_clearance < self.goal_tolerance:
            raise ValueError("Invalid target clearance")

        if (
            self.segment_delta > 0.6
            or self.path_speed > 0.5 / 1.5
        ):
            raise ValueError(
                "Trajectory parameters exceed MotionComponent limits"
            )

        checker = self.planner.collision_filter

        self.obstacle_names = sorted(
            checker.model.body(i).name
            for i in range(checker.model.nbody)
            if checker.model.body(i).name.startswith("obstacle_")
        )

        if mujoco.mj_name2id(
            checker.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "target",
        ) < 0:
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

        self.current_q = None
        self.goal_q = None
        self.current_path = None

        self.target_base = None
        self.last_target_world = None
        self.last_obstacle_poses = None

        self.feedback_time = None
        self.obstacle_time = None

        self.obstacles_ready = False
        self.target_ready = False

        self.planning_required = True
        self.graph_update_required = self.show_joint_graph

        self.status = None
        self.motion_status = None

        self.motion_command_pending = False
        self.execution_started = False

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

        # Motion commands must not replay after reconnection.
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

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10,
        )

        self.create_subscription(
            PoseArray,
            "/simulation/obstacle_poses",
            self.obstacle_callback,
            10,
        )

        self.create_subscription(
            Marker,
            "/simulation/target_marker",
            self.target_callback,
            10,
        )

        self.create_subscription(
            String,
            "/motion/execution_status",
            self.motion_status_callback,
            durable,
        )

        self.create_timer(
            0.2,
            self.planning_cycle,
        )

        self.set_status("WAITING_FOR_FEEDBACK")

        self.get_logger().info(
            "PlanningNode ready; "
            "goal_source=joint_eof, "
            f"execute_path={self.execute_path}"
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
            [
                positions[name]
                for name in self.joint_names
            ],
            dtype=float,
        )

        if not np.isfinite(joints).all():
            return

        self.current_q = joints
        self.feedback_time = time.monotonic()

    def motion_status_callback(self, message):
        self.motion_status = message.data

        if self.motion_status == "EXECUTING":
            self.execution_started = True

        if self.motion_status in (
            "FEEDBACK_TIMEOUT",
            "CONTROL_ERROR",
            "STOPPED",
        ):
            self.motion_command_pending = False

        if self.motion_status == "COMPLETED":
            self.motion_command_pending = False

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

    def cancel_execution(self):
        if self.execute_path:
            self.stop_pub.publish(Bool(data=True))

        self.motion_command_pending = False
        self.execution_started = False

    def invalidate_path(self, status, error=None):
        self.cancel_execution()

        self.current_path = None
        self.goal_q = None

        self.planning_required = False
        self.graph_update_required = False

        self.publish_preview(None)

        for kind in (
            "initial",
            "invalid",
            "replanned",
        ):
            self.visualizer.clear_path(kind)

        self.set_status(status)

        if error is not None:
            self.get_logger().error(str(error))

    def target_callback(self, message):
        if message.action != Marker.ADD:
            return

        if message.header.frame_id != self.base_frame:
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
            self.target_ready = False
            self.invalidate_path("TARGET_ERROR")
            return

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
            self.target_ready = False
            self.invalidate_path("TARGET_ERROR")
            return

        if (
            self.last_target_world is not None
            and np.allclose(
                world,
                self.last_target_world,
                atol=1e-6,
                rtol=0,
            )
        ):
            self.target_ready = True
            return

        self.cancel_execution()

        self.current_path = None
        self.goal_q = None

        self.publish_preview(None)

        try:
            self.planner.update_obstacle(
                "target",
                world,
            )
        except Exception as error:
            self.target_ready = False
            self.invalidate_path(
                "TARGET_ERROR",
                error,
            )
            return

        self.target_base = target.copy()
        self.last_target_world = world.copy()

        self.target_ready = True
        self.planning_required = True

        self.graph_update_required = (
            self.show_joint_graph
        )

    def obstacle_callback(self, message):
        if (
            message.header.frame_id != "mujoco_world"
            or len(message.poses) != len(self.obstacle_names)
        ):
            self.obstacles_ready = False

            self.invalidate_path(
                "OBSTACLE_ERROR",
                ValueError("Invalid obstacle array"),
            )
            return

        try:
            poses = [
                self.pose_arrays(pose)
                for pose in message.poses
            ]
        except ValueError as error:
            self.obstacles_ready = False
            self.invalidate_path(
                "OBSTACLE_ERROR",
                error,
            )
            return

        self.obstacle_time = time.monotonic()

        if self.last_obstacle_poses is not None:
            unchanged = all(
                np.allclose(
                    position,
                    old_position,
                    atol=1e-6,
                    rtol=0,
                )
                and abs(
                    float(
                        np.dot(
                            quaternion,
                            old_quaternion,
                        )
                    )
                ) > 1.0 - 1e-8
                for (
                    position,
                    quaternion,
                ), (
                    old_position,
                    old_quaternion,
                ) in zip(
                    poses,
                    self.last_obstacle_poses,
                )
            )

            if unchanged:
                self.obstacles_ready = True
                return

        self.cancel_execution()
        self.current_path = None
        self.publish_preview(None)

        try:
            for name, (
                position,
                quaternion,
            ) in zip(
                self.obstacle_names,
                poses,
            ):
                if np.allclose(
                    position,
                    [100, 100, 100],
                    atol=1e-5,
                    rtol=0,
                ):
                    self.planner.remove_obstacle(name)
                else:
                    self.planner.update_obstacle(
                        name,
                        position,
                        quaternion,
                    )
        except Exception as error:
            self.obstacles_ready = False
            self.invalidate_path(
                "OBSTACLE_ERROR",
                error,
            )
            return

        self.last_obstacle_poses = [
            (
                position.copy(),
                quaternion.copy(),
            )
            for position, quaternion in poses
        ]

        self.obstacles_ready = True
        self.planning_required = True

        self.graph_update_required = (
            self.show_joint_graph
        )

    def feedback_ready(self):
        return (
            self.current_q is not None
            and self.feedback_time is not None
            and time.monotonic() - self.feedback_time
            <= self.feedback_timeout
        )

    def obstacle_data_ready(self):
        return (
            self.obstacles_ready
            and self.obstacle_time is not None
            and time.monotonic() - self.obstacle_time
            <= self.obstacle_timeout
        )

    def select_goal(self):
        # Select a collision-free sampled goal near the target.
        distances = np.linalg.norm(
            self.joint_eof - self.target_base,
            axis=1,
        )

        candidates = np.argsort(distances)

        checker = self.planner.collision_filter

        for index in candidates:
            distance = float(distances[index])

            if distance > self.goal_tolerance:
                break

            if distance < self.target_clearance:
                continue

            joints = self.planner.joint_q[index]

            if checker.is_state_valid(joints):
                self.get_logger().info(
                    f"Selected goal node={index}, "
                    f"EOF error={distance:.3f} m"
                )

                return joints.copy()

        return None

    def prepare_trajectory(self, path):
        # Subdivide long A* edges without changing their geometry.
        result = [
            np.asarray(
                path[0],
                dtype=float,
            ).copy()
        ]

        for start, goal in zip(
            path[:-1],
            path[1:],
        ):
            delta = goal - start

            steps = max(
                1,
                int(
                    np.ceil(
                        np.max(np.abs(delta))
                        / self.segment_delta
                    )
                ),
            )

            for step in range(1, steps + 1):
                result.append(
                    start + delta * step / steps
                )

        result = np.asarray(result)

        if not self.planner.is_path_valid(result):
            raise ValueError(
                "Execution path failed collision validation"
            )

        return self.build_trajectory(result)

    def build_trajectory(self, path):
        message = JointTrajectory()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.joint_names = list(self.joint_names)

        elapsed = 0.0

        for index, joints in enumerate(path):
            if index:
                delta = float(
                    np.max(
                        np.abs(
                            joints - path[index - 1]
                        )
                    )
                )

                elapsed += max(
                    delta / self.path_speed,
                    0.01,
                )

            point = JointTrajectoryPoint()

            point.positions = joints.tolist()

            nanoseconds = round(elapsed * 1e9)

            point.time_from_start.sec = (
                nanoseconds // 1_000_000_000
            )

            point.time_from_start.nanosec = (
                nanoseconds % 1_000_000_000
            )

            message.points.append(point)

        return message

    def publish_preview(self, path):
        if path is None:
            message = JointTrajectory()

            message.header.stamp = (
                self.get_clock().now().to_msg()
            )

            message.joint_names = list(self.joint_names)
        else:
            message = self.build_trajectory(path)

        self.preview_pub.publish(message)

    def planning_cycle(self):
        if not self.feedback_ready():
            if (
                self.execute_path
                and self.motion_command_pending
            ):
                self.cancel_execution()

            self.set_status("WAITING_FOR_FEEDBACK")
            return

        if not self.obstacle_data_ready():
            if (
                self.execute_path
                and self.motion_command_pending
            ):
                self.cancel_execution()

            self.set_status("WAITING_FOR_OBSTACLES")
            return

        if not self.target_ready:
            self.set_status("WAITING_FOR_TARGET")
            return

        if self.planning_required:
            # Require a stopped controller before replanning.
            if (
                self.execute_path
                and self.motion_status != "STOPPED"
            ):
                self.cancel_execution()
                self.set_status("WAITING_FOR_STOP")
                return

            self.planning_required = False
            started = time.monotonic()

            try:
                goal = self.select_goal()

                if goal is None:
                    self.invalidate_path(
                        "GOAL_UNREACHABLE"
                    )
                else:
                    self.goal_q = goal

                    self.set_status("PLANNING")

                    if not self.planner.plan(
                        self.current_q.copy(),
                        goal,
                    ):
                        self.invalidate_path("NO_PATH")
                    else:
                        self.current_path = (
                            self.planner.current_path.copy()
                        )

                        self.visualizer.publish_path(
                            self.current_path,
                            "initial",
                        )

                        self.visualizer.clear_path("invalid")
                        self.visualizer.clear_path("replanned")

                        self.publish_preview(
                            self.current_path
                        )

                        self.set_status("PATH_READY")

            except Exception as error:
                self.invalidate_path(
                    "PLANNING_ERROR",
                    error,
                )

            self.get_logger().info(
                "Path planning took "
                f"{time.monotonic() - started:.2f} s"
            )

        if self.graph_update_required:
            self.graph_update_required = False

            if self.show_joint_graph:
                try:
                    self.visualizer.publish_filtered_graph(
                        self.planner.get_filtered_graph()
                    )
                except Exception as error:
                    self.get_logger().error(
                        "Graph visualization failed: "
                        f"{error}"
                    )

        if (
            self.execute_path
            and self.current_path is not None
            and not self.execution_started
        ):
            self.try_execute()

    def try_execute(self):
        if self.motion_command_pending:
            if self.motion_status == "IDLE":
                self.motion_command_pending = False
            else:
                return

        if self.motion_status == "STOPPED":
            self.stop_pub.publish(
                Bool(data=False)
            )

            self.motion_command_pending = True
            self.set_status("WAITING_FOR_RELEASE")
            return

        if self.motion_status != "IDLE":
            return

        if (
            not self.feedback_ready()
            or not self.obstacle_data_ready()
        ):
            return

        start_error = float(
            np.max(
                np.abs(
                    self.current_q
                    - self.current_path[0]
                )
            )
        )

        if start_error > 0.05:
            self.current_path = None
            self.planning_required = True

            self.stop_pub.publish(
                Bool(data=True)
            )
            return

        path = self.current_path.copy()
        path[0] = self.current_q.copy()

        try:
            trajectory = self.prepare_trajectory(path)
        except Exception as error:
            self.invalidate_path(
                "TRAJECTORY_ERROR",
                error,
            )
            return

        if not self.feedback_ready():
            return

        self.motion_pub.publish(trajectory)

        self.execution_started = True
        self.set_status("TRAJECTORY_SENT")


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