#!/usr/bin/env python3

import json
from pathlib import Path
import time

import mujoco
import numpy as np
import rclpy
import yaml

from geometry_msgs.msg import PoseArray
from msgs.msg import HumanCapsuleArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from planning.human_capsule_monitor import HumanCapsuleMonitor
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
            "simulation_only_ignore_unknown": True,
            "profile_planning": True,
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

        human_config = config.get("planning", {}).get(
            "human_avoidance", {}
        )

        self.human_enabled = bool(
            human_config.get("enabled", False)
        )

        human_timeout = float(
            human_config.get("data_timeout", 0.5)
        )

        human_future_tolerance = float(
            human_config.get("future_tolerance", 0.1)
        )

        human_safety_margin = float(
            human_config.get("safety_margin", 0.10)
        )

        if (
            not np.isfinite(human_timeout)
            or human_timeout <= 0
            or not np.isfinite(human_future_tolerance)
            or human_future_tolerance < 0
            or not np.isfinite(human_safety_margin)
            or human_safety_margin < 0
        ):
            raise ValueError("Invalid human avoidance configuration")

        self.planner = PathPlanner(
            paths["graph_path"],
            paths["model_path"],
            paths["config_path"],
        )

        checker = self.planner.collision_filter
        checker.human_data_timeout = human_timeout
        checker.human_safety_margin = human_safety_margin

        self.manager = PlanningManager(
            path_planner=self.planner,
            goal_tolerance=get("goal_tolerance"),
            target_clearance=get("target_clearance"),
            feedback_timeout=get("feedback_timeout"),
            obstacle_timeout=get("obstacle_timeout"),
            execute_path=bool(get("execute_path")),
            human_avoidance_enabled=self.human_enabled,
            simulation_only_ignore_unknown=bool(get("simulation_only_ignore_unknown")),
            human_margin_recovery_enabled=robot.get("mode") == "simulation",
            profiling_enabled=bool(get("profile_planning")),
        )

        self.human_monitor = HumanCapsuleMonitor(
            frame_id=self.base_frame,
            data_timeout=human_timeout,
            future_tolerance=human_future_tolerance,
        )

        self.last_human_report = None
        self.last_collision_report = None
        self.last_manager_error = None
        self.last_runtime_report_time = None
        self.last_profile_report_time = None
        self.last_profile_decision = None
        self.last_human_callback_profile = None
        self.last_path_failure_id = 0

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

        self.human_sub = self.create_subscription(
            HumanCapsuleArray,
            "/vision/human/geometry",
            self.human_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        self.timer = self.create_timer(
            0.2,
            self.planning_cycle,
        )

        self.set_status("WAITING_FOR_FEEDBACK")

        self.get_logger().info(
            "PlanningNode initialized; "
            f"execute_path={self.manager.execute_path}, "
            f"human_avoidance={self.human_enabled}, "
            f"human_data_timeout={human_timeout:.3f}s, "
            f"human_safety_margin={human_safety_margin:.3f}m, "
            f"human_geometry_cache={checker.human_report_cache_size}, "
            f"profile_planning={self.manager.profiler.enabled}, "
            f"segment_progress={self.planner.segment_progress_enabled}, "
            f"progress_corridor="
            f"{self.planner.execution_progress_config['corridor_tolerance']}rad, "
            f"simulation_only_ignore_unknown="
            f"{self.manager.simulation_only_ignore_unknown}, "
            f"human_margin_recovery={self.manager.human_margin_recovery_enabled}"
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

        velocities = None
        if message.velocity:
            if len(message.velocity) != len(message.name):
                return
            named_velocities = dict(zip(message.name, message.velocity))
            velocities = [named_velocities[name] for name in self.joint_names]
        self.manager.update_joint_state(joints, velocities)

    def human_callback(self, message):
        now_ros_ns = self.get_clock().now().nanoseconds

        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

        age_ms = (now_ros_ns - stamp_ns) / 1e6

        accepted = self.human_monitor.update(
            message,
            now_ros_ns,
        )

        if accepted:
            self.last_human_callback_profile = (stamp_ns, age_ms)

        if not accepted:
            self.get_logger().warning(
                f"Human geometry rejected: "
                f"reason={self.human_monitor.reason}, "
                f"age={age_ms:.1f} ms, "
                f"valid={message.valid}"
            )

    def monitor_humans(self):
        snapshot = self.human_monitor.snapshot(
            self.get_clock().now().nanoseconds
        )

        self.manager.update_human_snapshot(snapshot)

        report = (
            snapshot.status,
            snapshot.reason,
            snapshot.human_count,
            len(snapshot.capsules),
        )

        if report == self.last_human_report:
            return

        self.last_human_report = report

        self.get_logger().info(
            f"Human Vision: {snapshot.status} "
            f"({snapshot.reason}), "
            f"humans={snapshot.human_count}, "
            f"capsules={len(snapshot.capsules)}"
        )

    def motion_status_callback(self, message):
        self.manager.update_motion_status(message.data)

    def report_collision_diagnostics(self):
        """Log changed collision conditions and previously hidden exceptions."""
        manager = self.manager
        report = manager.human_report
        blocked = report.get("status") != "FAR"
        retreat_reason = (
            manager.human_margin_block_reason
            if manager.status == "HUMAN_RETREAT_BLOCKED" else None
        )
        key = (
            report.get("detection_status"), report.get("status"), report.get("reason"),
            report.get("robot_geom") if blocked else None,
            report.get("body_id") if blocked else None,
            report.get("segment_id") if blocked else None,
            retreat_reason,
        )
        if key != self.last_collision_report:
            self.last_collision_report = key
            self.get_logger().info(
                f"Human collision: detection={report.get('detection_status')}, "
                f"revision={report.get('revision')}, "
                f"q={report.get('joints')}, status={report.get('status')}, "
                f"reason={report.get('reason')}, "
                f"retreat_reason={retreat_reason}, "
                f"clearance={report.get('clearance')}, "
                f"geometry_model={report.get('geometry_model')}, "
                f"sphere_clearance={report.get('sphere_clearance')}, "
                f"box_clearance={report.get('box_clearance')}, "
                f"robot_geom={report.get('robot_geom')}, "
                f"robot_radius={report.get('robot_radius')}, "
                f"body_id={report.get('body_id')}, "
                f"segment_id={report.get('segment_id')}, "
                f"human_radius={report.get('human_radius')}, "
                f"safety_margin={self.planner.collision_filter.human_safety_margin}"
            )

        if manager.last_error != self.last_manager_error:
            self.last_manager_error = manager.last_error
            if manager.last_error is not None:
                self.get_logger().error(manager.last_error)

    def failure_reason_callback(self, message):
        self.manager.update_failure_reason(message.data)

    def report_runtime_diagnostics(self):
        """Keep an unchanged STOP/fault and target offsets visible in logs."""
        now = self.get_clock().now().nanoseconds
        previous = self.last_runtime_report_time
        if previous is not None and 0 <= now - previous < 5_000_000_000:
            return
        self.last_runtime_report_time = now
        manager = self.manager
        target, actual, goal = manager.target_base, None, None
        try:
            if manager.goal_q is not None:
                goal = self.visualizer.forward_kinematics(manager.goal_q)
            if manager.current_q is not None:
                # Leave the visualization scratch model at the measured pose.
                actual = self.visualizer.forward_kinematics(manager.current_q)
        except Exception as error:
            self.get_logger().warning(f"Target distance diagnostic unavailable: {error}")
        actual_error = None if actual is None or target is None else float(
            np.linalg.norm(actual - target))
        goal_error = None if goal is None or target is None else float(
            np.linalg.norm(goal - target))
        checker = self.planner.collision_filter
        self.get_logger().info(
            f"Planning snapshot: status={manager.status}, motion={manager.motion_status}, "
            f"failure_reason={manager.failure_reason}, fault={manager.fault}, "
            f"stop_owned={manager.stop_owned}, pending={manager.pending_action}, "
            f"planning_required={manager.planning_required}, "
            f"path_points={0 if manager.current_path is None else len(manager.current_path)}, "
            f"feedback_ready={manager.feedback_ready()}, "
            f"obstacles_ready={manager.obstacle_data_ready()}, "
            f"human={manager.human_status}/{manager.human_reason}, "
            f"human_checks={checker.human_obstacles_enabled}, "
            f"human_validation_pending={manager.human_validation_pending}, "
            f"path_validation={manager.human_path_validation_reason}, "
            f"next_waypoint={manager.execution_next_waypoint}, "
            f"plan_failure={self.planner.last_plan_failure}, "
            f"target={None if target is None else target.tolist()}, "
            f"eof={None if actual is None else actual.tolist()}, "
            f"target_error_m={actual_error}, selected_goal_error_m={goal_error}, "
            f"goal_tolerance_m={manager.goal_tolerance}, "
            f"target_clearance_m={manager.target_clearance}"
        )

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

    def report_path_failure(self):
        """Report frozen failed-check evidence after dispatch, without geometry work."""
        report = self.manager.last_path_failure
        if report is None or report["id"] == self.last_path_failure_id:
            return
        self.last_path_failure_id = report["id"]
        self.get_logger().info(
            "Path validation failure: " + json.dumps(
                report, separators=(",", ":"), allow_nan=False))

    def report_planning_profile(self, decision, began, observation):
        """Emit after command dispatch; nested stage durations are inclusive."""
        profiler = self.manager.profiler
        if not profiler.enabled:
            return
        now = time.monotonic()
        work_ms = (time.perf_counter() - began) * 1000.
        key = (decision.action, decision.status)
        acknowledgement = any('origin_cycle' in row for row in profiler.records)
        failed_stage = any(row['outcome'] == 'ERROR' for row in profiler.records)
        if (
            key == self.last_profile_decision and decision.action == "NONE"
            and work_ms < 100. and not acknowledgement and not failed_stage
            and self.last_profile_report_time is not None
            and now - self.last_profile_report_time < 5.
        ):
            return
        self.last_profile_decision = key
        self.last_profile_report_time = now
        checker = self.planner.collision_filter
        payload = {
            "cycle": profiler.cycle,
            "status": decision.status,
            "action": decision.action,
            "cycle_work_ms": round(work_ms, 3),
            "human_status": self.manager.human_status,
            "human_reason": self.manager.human_reason,
            "human_timeout_ms": checker.human_data_timeout * 1000.,
            "human_geometry_cache": {
                "capacity": getattr(checker, "human_report_cache_size", 0),
                "entries": len(getattr(checker, "_human_report_cache", {})),
                "hits_total": getattr(checker, "human_report_cache_hits", 0),
                "misses_total": getattr(checker, "human_report_cache_misses", 0),
            },
            "path_validation": self.manager.human_path_validation_reason,
            "path_failure_id": self.manager.path_failure_sequence,
            "commanded_path_id": self.manager.execution_path_id,
            "next_waypoint": self.manager.execution_next_waypoint,
            "candidate_next_waypoint": self.manager._candidate_next_waypoint(),
            "progress_reason": self.manager.execution_progress_reason,
            "segment_evidence_reason": (
                None if self.manager.segment_progress is None
                else self.manager.segment_progress.reason),
            "retreat_reason": (
                self.manager.human_margin_block_reason
                if decision.status == "HUMAN_RETREAT_BLOCKED" else None),
            **observation,
            "stages": list(profiler.records),
        }
        self.get_logger().info(
            "Planning profile: " + json.dumps(payload, separators=(",", ":"), allow_nan=False))

    def planning_cycle(self):
        began = time.perf_counter()
        stamp_ns = self.human_monitor.stamp_ns
        previous_callback = self.last_human_callback_profile
        observation = {
            "human_capture_stamp_ns": stamp_ns,
            "capture_age_start_ms": (
                round((self.get_clock().now().nanoseconds - stamp_ns) / 1e6, 3)
                if stamp_ns is not None else None),
            "capture_to_callback_ms": (
                round(previous_callback[1], 3)
                if previous_callback is not None and previous_callback[0] == stamp_ns
                else None),
        }
        self.monitor_humans()

        decision = self.manager.step()
        self.report_collision_diagnostics()

        self.set_status(decision.status)
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
                self.get_logger().error(
                    "Execution decision has no path"
                )
                self.stop_pub.publish(Bool(data=True))
                self.report_path_failure()
                self.report_planning_profile(decision, began, observation)
                return

            try:
                trajectory = self.manager.profiler.call(
                    "trajectory_build", self.build_trajectory, decision.path)

            except ValueError as error:
                self.get_logger().error(
                    f"Trajectory rejected: {error}"
                )
                self.stop_pub.publish(Bool(data=True))
                self.report_path_failure()
                self.report_planning_profile(decision, began, observation)
                return

            self.motion_pub.publish(trajectory)

            self.get_logger().info(
                f"{decision.status}: "
                f"{len(trajectory.points)} waypoints sent"
            )

        # Dispatch Motion commands before potentially expensive visualization.
        self.report_path_failure()
        self.report_runtime_diagnostics()
        self.manager.profiler.call("visualization", self.update_visualization)
        self.manager.profiler.call("graph_visualization", self.update_graph_visualization)
        self.report_planning_profile(decision, began, observation)


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
