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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
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

        # Parameter path executor (meter).
        self.waypoint_tolerance = 0.025
        self.path_lookahead = 0.035
        self.path_cross_track_tolerance = 0.035
        self.progress_timeout = 3.0
        self.progress_epsilon = 0.003
        self.best_waypoint_distance = float("inf")
        self.last_progress_time = time.monotonic()
        self.last_stall_warning = 0.0

        # Optional waypoint bypass when robot stalls against joint limits.
        # This does NOT prove that bypass is collision-free.
        self.skip_on_limit_stall = True
        self.skip_stall_seconds = 3.0
        self.skip_cross_track_max = 0.065  # diagnostic cap, not a safety proof
        self.skip_endpoint_max = 0.15      # meters
        self.joint_limit_margin = 0.005     # radians
        self.last_dq_command = np.zeros_like(self.q_des)
        self.skip_request_pending = False
        self.skip_request_time = 0.0
        self.skip_candidate = None
        self.last_replan_request = 0.0
        self.replan_cooldown = 5.0
        self.executor_paused = False
        self.failed_plan_signatures = set()
        self.path_frame_id = "base_link"


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

        # The planner owns obstacle geometry. Never bypass without its approval.
        self.skip_validation_publisher = self.create_publisher(
            PathMessage, "/planning/validate_skip", 10
        )
        self.skip_validation_subscriber = self.create_subscription(
            PathMessage, "/planning/skip_validation_result",
            self.skip_validation_callback, 10
        )
        self.replan_publisher = self.create_publisher(
            Empty, "/planning/request_replan", 10
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
            self.best_waypoint_distance = float("inf")

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

        received_path = np.asarray(path_positions, dtype=np.float64)
        self.path_frame_id = message.header.frame_id or "base_link"
        # Avoid endless replanning of precisely the same non-executable path.
        signature = tuple(np.round(received_path.flatten(), 3))
        if self.executor_paused and signature in self.failed_plan_signatures:
            self.get_logger().error(
                "Planner returned a previously blocked path. Execution stays paused. "
                "A different path or joint-space planning is required."
            )
            return
        self.executor_paused = False
        self.skip_request_pending = False
        self.skip_candidate = None
        self.path = received_path

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
        self.best_waypoint_distance = float("inf")
        self.last_progress_time = time.monotonic()
        self.last_stall_warning = 0.0

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

    def outward_limited_joints(self):
        """Joint ids currently at a limit with a command pointing outward."""
        q = self.kinematics.get_joint_positions()
        dq = self.last_dq_command
        at_lower = (q <= self.q_lower + self.joint_limit_margin) & (dq < -1e-4)
        at_upper = (q >= self.q_upper - self.joint_limit_margin) & (dq > 1e-4)
        return np.flatnonzero(at_lower | at_upper).tolist()

    def request_replan(self, reason):
        now = time.monotonic()
        if now - self.last_replan_request < self.replan_cooldown:
            return
        self.last_replan_request = now
        self.executor_paused = True
        if self.path is not None:
            self.failed_plan_signatures.add(tuple(np.round(self.path.flatten(), 3)))
        self.get_logger().warning(f"PAUSED; requesting replanning: {reason}")
        self.replan_publisher.publish(Empty())

    def request_skip_validation(self, current_eof):
        if self.skip_request_pending or self.path_index >= len(self.path) - 1:
            return
        # Candidate shortcut starts at the ACTUAL EoF, not the nominal waypoint.
        # Do not skip the final goal. Let the planner check target + obstacle.
        end = self.path[self.path_index + 1]
        if np.linalg.norm(end - current_eof) > 0.25:
            self.request_replan("Next waypoint too far for a local bypass")
            return
        request = PathMessage()
        request.header.frame_id = self.path_frame_id
        for xyz in (current_eof, end):
            pose = PoseStamped()
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0
            request.poses.append(pose)
        self.skip_candidate = (self.path_index, current_eof.copy(), end.copy())
        self.skip_request_pending = True
        self.skip_request_time = time.monotonic()
        self.executor_paused = True
        self.skip_validation_publisher.publish(request)
        self.get_logger().warning(
            f"STALLED at waypoint {self.path_index}; asking planner to "
            "validate a shortcut from the actual EoF."
        )

    def skip_validation_callback(self, message):
        if not self.skip_request_pending or self.skip_candidate is None:
            return
        index, start, end = self.skip_candidate
        self.skip_request_pending = False
        self.skip_candidate = None
        # Result carries the same two poses only when the segment is clear.
        accepted = len(message.poses) == 2 and message.header.frame_id == self.path_frame_id
        if accepted:
            returned = np.array([
                [p.pose.position.x, p.pose.position.y, p.pose.position.z]
                for p in message.poses
            ])
            accepted = np.allclose(returned, [start, end], atol=1e-4)
        # EoF may have moved while validating. Never use a stale approval.
        current = self.get_current_eof_robot()
        if accepted and np.linalg.norm(current - start) <= 0.01 and self.path_index == index:
            self.path_index += 1
            # New segment starts at the actually validated EoF, not the skipped point.
            self.path[self.path_index - 1] = start.copy()
            self.current_waypoint = self.path[self.path_index].copy()
            self.best_waypoint_distance = float('inf')
            self.last_progress_time = time.monotonic()
            self.executor_paused = False
            self.q_des = np.clip(self.kinematics.get_joint_positions().copy(),
                                 self.q_lower, self.q_upper)
            self.get_logger().warning(
                f"BYPASS APPROVED: waypoint {index} skipped after planner "
                "validated the direct segment to next waypoint."
            )
        else:
            self.request_replan("Shortcut unsafe, stale, or rejected")

    def update_waypoint(self, current_eof):
        """Advance only after reaching/passing the current segment endpoint.

        Works in the same robot base frame as the Path message.
        No blind skipping on timeout: that could cross an obstacle.
        """
        if not self.path_active or self.path is None:
            return

        # Skip consecutive near-duplicate points safely.
        while self.path_active:
            end = self.path[self.path_index]
            start = self.path[self.path_index - 1]
            segment = end - start
            length_sq = float(np.dot(segment, segment))
            dist = float(np.linalg.norm(end - current_eof))

            if length_sq > 1e-12:
                projection = float(np.dot(current_eof - start, segment) / length_sq)
                closest = start + np.clip(projection, 0.0, 1.0) * segment
                cross_track = float(np.linalg.norm(current_eof - closest))
            else:
                projection = 1.0
                cross_track = dist

            # Waypoint is reached either by proximity or by safely passing it
            # while staying close to the intended line segment.
            reached = (dist <= self.waypoint_tolerance or
                       (projection >= 1.0 and
                        cross_track <= self.path_cross_track_tolerance))
            if not reached:
                now = time.monotonic()
                if dist < self.best_waypoint_distance - self.progress_epsilon:
                    self.best_waypoint_distance = dist
                    self.last_progress_time = now

                stalled = now - self.last_progress_time >= self.skip_stall_seconds
                limited = self.outward_limited_joints() if stalled else []
                has_next = self.path_index < len(self.path) - 1
                # A limit by itself is not sufficient; demand a real stall.
                # Also detect severe actuator tracking error when no limit is reported.
                q_actual = self.kinematics.get_joint_positions()
                joint_error = float(np.max(np.abs(self.q_des - q_actual)))
                blocked = bool(limited) or joint_error >= 0.12
                if (self.skip_on_limit_stall and stalled and blocked
                        and has_next and not self.executor_paused):
                    self.request_skip_validation(current_eof)
                elif stalled and not has_next and not self.executor_paused:
                    self.request_replan("Final waypoint cannot be reached")
                if self.skip_request_pending and now - self.skip_request_time > 2.0:
                    self.skip_request_pending = False
                    self.skip_candidate = None
                    self.request_replan("Skip validation timed out")

                if (now - self.last_progress_time >= self.progress_timeout
                      and now - self.last_stall_warning >= self.progress_timeout):
                    self.get_logger().warning(
                        f"Executor not progressing: waypoint "
                        f"{self.path_index}/{len(self.path)-1}, "
                        f"distance={dist*1000:.1f} mm, "
                        f"cross_track={cross_track*1000:.1f} mm. "
                        "Check reachability, joint saturation, collisions and control error."
                    )
                    self.last_stall_warning = now
                break

            self.get_logger().info(
                f"Waypoint reached: {self.path_index}/{len(self.path)-1}, "
                f"error={dist*1000:.1f} mm"
            )
            self.path_index += 1
            self.best_waypoint_distance = float("inf")
            self.last_progress_time = time.monotonic()

            if self.path_index >= len(self.path):
                self.path_active = False
                self.current_waypoint = None
                self.get_logger().info("A* path completed.")
                break
            self.current_waypoint = self.path[self.path_index].copy()

    def get_control_target(self, current_eof=None):
        """A small lookahead along the current A* segment avoids point-chasing.

        Does not use a smoothed trajectory because it has not been collision
        validated; small lookahead limits corner cutting around obstacles.
        """
        if not self.path_active or self.path is None:
            return self.get_target_position_robot()

        end = self.path[self.path_index]
        if current_eof is None:
            return end.copy()
        start = self.path[self.path_index - 1]
        segment = end - start
        seg_length = float(np.linalg.norm(segment))
        if seg_length < 1e-9:
            return end.copy()

        direction = segment / seg_length
        along = float(np.dot(current_eof - start, direction))
        follow_distance = np.clip(along + self.path_lookahead, 0.0, seg_length)
        return start + follow_distance * direction

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
        vmax=5.0,
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
                        tip_robot.copy() if self.executor_paused
                        else self.get_control_target(tip_robot)
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

                    if self.executor_paused:
                        dq[:] = 0.0
                        self.q_des = self.kinematics.get_joint_positions().copy()
                    self.last_dq_command = dq.copy()
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