#!/usr/bin/env python3

import time
from dataclasses import dataclass

import numpy as np


@dataclass
class PlanningDecision:
    action: str = "NONE"
    status: str = "IDLE"
    path: np.ndarray | None = None


class PlanningManager:
    def __init__(
        self,
        path_planner,
        goal_tolerance=0.12,
        target_clearance=0.04,
        feedback_timeout=1.0,
        obstacle_timeout=1.0,
        command_timeout=2.0,
        execute_path=False,
        recovery_settle_time=0.5,
        recovery_stability_tolerance=0.01,
        human_avoidance_enabled=False,
        simulation_only_ignore_unknown=False,
    ):
        self.planner = path_planner

        self.goal_tolerance = float(goal_tolerance)
        self.target_clearance = float(target_clearance)
        self.feedback_timeout = float(feedback_timeout)
        self.obstacle_timeout = float(obstacle_timeout)
        self.command_timeout = float(command_timeout)
        self.execute_path = bool(execute_path)
        self.recovery_settle_time = float(recovery_settle_time)
        self.recovery_stability_tolerance = float(
            recovery_stability_tolerance
        )

        self.human_avoidance_enabled = bool(human_avoidance_enabled)
        self.human_status = "UNKNOWN"
        self.human_revision = None
        self.simulation_only_ignore_unknown = bool(
            simulation_only_ignore_unknown
        )
        self.human_replan_blocked = False
        self.human_retry_count = 0
        self.human_retry_revision = None
        self.safe_position_active = False
        self.safe_position_attempted_revision = None

        values = (
            self.goal_tolerance,
            self.feedback_timeout,
            self.obstacle_timeout,
            self.command_timeout,
            self.recovery_settle_time,
            self.recovery_stability_tolerance,
        )

        if any(not np.isfinite(v) or v <= 0 for v in values):
            raise ValueError("Invalid PlanningManager configuration")

        if not 0 <= self.target_clearance < self.goal_tolerance:
            raise ValueError("Invalid target clearance")

        self.current_q = None
        self.target_base = None
        self.goal_q = None
        self.current_path = None

        self.feedback_time = None
        self.obstacle_time = None
        self.obstacles_ready = False
        self.last_obstacles = {}

        self.motion_status = None
        self.status = "WAITING_FOR_FEEDBACK"

        self.planning_required = True
        self.path_sent = False
        self.motion_active = False

        self.pending_action = None
        self.pending_since = None

        self.stop_owned = False
        self.fault = False

        self.failure_reason = "NONE"
        self.awaiting_failure_reason_since = None

        self.recovery_pending = False
        self.recovery_attempted = False
        self.recovery_start_q = None
        self.recovery_start_time = None

        self.failed_execution_path = None
        self.last_execution_path = None
        self.resume_after_fault = False

        self.recovery_blocked = False
        self.safe_recovery_active = False
        self.safe_recovery_attempted = False
        self.safe_recovery_revision = None
        self.safe_recovery_initial_state = None

        if self.human_avoidance_enabled:
            self.planner.collision_filter.enable_human_obstacles(True)

    def update_human_snapshot(self, snapshot):
        """Update human geometry without triggering replanning."""

        checker = self.planner.collision_filter
        checker.set_human_capsules(snapshot)
        self.human_status = checker.human_status

    def _human_guard(self):
        """Validate human proximity and the currently commanded path."""

        checker = self.planner.collision_filter
        ready = checker.human_data_ready()

        if not ready:
            # Permitted only for an isolated MuJoCo experiment.
            # Physical deployments must keep this setting disabled.
            if self.simulation_only_ignore_unknown:
                checker.enable_human_obstacles(False)
                self.human_revision = checker.environment_revision
                if self.human_replan_blocked and not self._stop_required():
                    self.human_replan_blocked = False
                    self.planning_required = True
                return None

            if self.current_path is not None or self.path_sent:
                self.invalidate_path()
            return "HUMAN_VISION_UNKNOWN"

        if self.simulation_only_ignore_unknown:
            checker.enable_human_obstacles(True)

        if not self.feedback_ready():
            if self.current_path is not None or self.path_sent:
                self.invalidate_path()
            return "WAITING_FOR_FEEDBACK"

        try:
            report = checker.human_collision_report(self.current_q)
        except Exception:
            report = {"status": "UNKNOWN"}

        if report["status"] == "UNKNOWN":
            if self.current_path is not None or self.path_sent:
                self.invalidate_path()
            return "HUMAN_VISION_UNKNOWN"

        if report["status"] == "NEAR":
            if self._stop_required():
                self.invalidate_path()
                self.human_replan_blocked = True
                return "HUMAN_TOO_CLOSE"

            # After STOPPED, plan from the measured joint configuration.
            # A colliding start cannot be used for an ordinary trajectory.
            if not checker.is_state_valid(self.current_q):
                self.invalidate_path()
                self.human_replan_blocked = True
                return "HUMAN_TOO_CLOSE"

            if self.current_path is not None:
                if checker.is_path_valid(self.current_path):
                    self.human_revision = checker.environment_revision
                    return None
                self.invalidate_path()
                self.human_replan_blocked = True

            if self.human_replan_blocked:
                self.planning_required = True
                self.human_replan_blocked = False
            self.human_revision = checker.environment_revision
            return None

        revision = checker.environment_revision
        if revision == self.human_revision:
            return None

        active = self._stop_required()
        path = self.last_execution_path if active else self.current_path

        if path is None:
            # Replan a previously blocked target when human geometry changes.
            if self.human_replan_blocked and not active:
                self.planning_required = True
                self.human_replan_blocked = False
            elif active:
                self.invalidate_path()
                self.human_replan_blocked = True
                return "HUMAN_PATH_BLOCKED"

            self.human_revision = revision
            return None

        try:
            valid = (
                checker.is_state_valid(self.current_q)
                and checker.is_path_valid(path)
                and checker.environment_revision == revision
                and checker.human_data_ready()
            )
        except Exception:
            valid = False

        if not valid:
            self.invalidate_path()
            self.human_replan_blocked = True
            return "HUMAN_PATH_BLOCKED"

        self.human_revision = revision
        return None

    def update_joint_state(self, joints):
        joints = np.asarray(joints, dtype=float)

        if (
            joints.shape != (5,)
            or not np.isfinite(joints).all()
        ):
            return False

        self.current_q = joints.copy()
        self.feedback_time = time.monotonic()

        return True

    def update_target(self, target_base, target_world):
        target_base = np.asarray(target_base, dtype=float)
        target_world = np.asarray(target_world, dtype=float)

        if (
            target_base.shape != (3,)
            or target_world.shape != (3,)
            or not np.isfinite(target_base).all()
            or not np.isfinite(target_world).all()
        ):
            raise ValueError("Invalid target position")

        if (
            self.target_base is not None
            and np.allclose(
                target_base,
                self.target_base,
                atol=1e-6,
                rtol=0.0,
            )
        ):
            return False

        self.planner.update_obstacle("target", target_world)

        self.target_base = target_base.copy()

        if not self.safe_recovery_active:
            self.recovery_blocked = False
            self.safe_recovery_attempted = False

        if (
            self.fault
            and self.motion_status == "CONTROL_ERROR"
            and self.failure_reason in (
                "GOAL_TIMEOUT",
                "TRACKING_ERROR",
                "STALLED",
            )
        ):
            self.fault = False
            self.resume_after_fault = True
            self.stop_owned = True
            self.recovery_pending = False
            self.awaiting_failure_reason_since = None
            self.recovery_attempted = False
            self.failed_execution_path = None
            self.last_execution_path = None

        elif not self.fault and not self.recovery_pending:
            self.recovery_attempted = False

        if self.safe_recovery_active:
            self.safe_recovery_active = False

        self.safe_position_active = False
        self.human_retry_count = 0
        self.safe_position_attempted_revision = None
        self.invalidate_path()
        return True

    def update_obstacles(self, obstacles):
        updates = {}

        for name, position, quaternion, removed in obstacles:
            if name in updates:
                raise ValueError(f"Duplicate obstacle: {name}")

            position = np.asarray(position, dtype=float)
            quaternion = np.asarray(quaternion, dtype=float)

            if (
                position.shape != (3,)
                or quaternion.shape != (4,)
                or not np.isfinite(position).all()
                or not np.isfinite(quaternion).all()
            ):
                raise ValueError(f"Invalid obstacle pose: {name}")

            norm = float(np.linalg.norm(quaternion))

            if not np.isfinite(norm) or norm < 1e-8:
                raise ValueError(f"Invalid obstacle rotation: {name}")

            updates[name] = (
                position.copy(),
                quaternion / norm,
                bool(removed),
            )

        if (
            self.last_obstacles
            and set(updates) != set(self.last_obstacles)
        ):
            raise ValueError("Obstacle set changed unexpectedly")

        changed_names = []

        for name, (position, quaternion, removed) in updates.items():
            previous = self.last_obstacles.get(name)

            if previous is None:
                changed_names.append(name)
                continue

            old_position, old_quaternion, old_removed = previous

            changed = (
                old_removed != removed
                or not np.allclose(
                    position,
                    old_position,
                    atol=1e-6,
                    rtol=0.0,
                )
                or abs(
                    float(np.dot(quaternion, old_quaternion))
                ) <= 1.0 - 1e-8
            )

            if changed:
                changed_names.append(name)

        for name in changed_names:
            position, quaternion, removed = updates[name]

            if removed:
                self.planner.remove_obstacle(name)
            else:
                self.planner.update_obstacle(
                    name,
                    position,
                    quaternion,
                )

        self.last_obstacles = updates
        self.obstacles_ready = True
        self.obstacle_time = time.monotonic()

        if changed_names:
            self.safe_recovery_attempted = False
            self.recovery_blocked = False

            if not self.fault and not self.recovery_pending:
                self.recovery_attempted = False

            self.invalidate_path()

        return bool(changed_names)

    def update_failure_reason(self, reason):
        self.failure_reason = str(reason)

        if (
            self.motion_status == "CONTROL_ERROR"
            and self.failure_reason != "NONE"
        ):
            self._classify_motion_error()

    def _classify_motion_error(self):
        recoverable = (
            self.motion_status == "CONTROL_ERROR"
            and self.failure_reason in (
                "GOAL_TIMEOUT",
                "TRACKING_ERROR",
                "STALLED",
            )
            and not self.recovery_attempted
            and self.execute_path
        )

        self.awaiting_failure_reason_since = None

        if recoverable:
            self.recovery_pending = True
            self.recovery_attempted = True
            self.recovery_start_q = None
            self.recovery_start_time = None
        else:
            self.fault = True
            self.recovery_pending = False

    def update_motion_status(self, status):
        self.motion_status = status

        if status in (
            "CONTROL_ERROR",
            "INVALID_FEEDBACK",
            "FEEDBACK_TIMEOUT",
        ):
            self.motion_active = False
            self.pending_action = None
            self.pending_since = None

            self.failed_execution_path = (
                None
                if self.last_execution_path is None
                else self.last_execution_path.copy()
            )

            self.invalidate_path()
            self.safe_recovery_active = False

            if (
                status == "CONTROL_ERROR"
                and self.failure_reason == "NONE"
            ):
                self.awaiting_failure_reason_since = time.monotonic()
            else:
                self._classify_motion_error()

            return

        if status == "STOPPED":
            commanded_stop = self.pending_action == "STOP"

            self.motion_active = False
            self.pending_action = None
            self.pending_since = None
            self.path_sent = False

            if not commanded_stop:
                self.safe_recovery_active = False
                self.invalidate_path()

            return

        if status == "IDLE":
            if self.pending_action == "RELEASE":
                self.pending_action = None
                self.pending_since = None
                self.stop_owned = False
                self.recovery_pending = False
                self.resume_after_fault = False

            return

        if status == "EXECUTING":
            if self.pending_action == "EXECUTE":
                self.pending_action = None
                self.pending_since = None
                self.motion_active = True

            return

        if status == "COMPLETED":
            if (
                self.pending_action == "EXECUTE"
                or self.motion_active
            ):
                self.pending_action = None
                self.pending_since = None
                self.motion_active = False
                self.path_sent = True

                self.last_execution_path = None
                self.failed_execution_path = None
                self.recovery_attempted = False

                if self.safe_position_active:
                    self.safe_position_active = False
                    self.human_retry_count = 0
                    self.invalidate_path()

                if self.safe_recovery_active:
                    self.safe_recovery_active = False
                    self.invalidate_path()
                    self.recovery_blocked = False

    def _recovery_step(self, now):
        if (
            not self.feedback_ready()
            or not self.obstacle_data_ready()
        ):
            return self._decision(
                status="RECOVERY_WAITING_FOR_FEEDBACK"
            )

        if self.target_base is None:
            self.fault = True
            return self._decision(status="RECOVERY_NO_TARGET")

        if self.recovery_start_q is None:
            self.recovery_start_q = self.current_q.copy()
            self.recovery_start_time = now

            return self._decision(status="RECOVERY_SETTLING")

        movement = float(
            np.max(
                np.abs(
                    self.current_q - self.recovery_start_q
                )
            )
        )

        if movement > self.recovery_stability_tolerance:
            self.recovery_start_q = self.current_q.copy()
            self.recovery_start_time = now

            return self._decision(status="RECOVERY_SETTLING")

        if now - self.recovery_start_time < self.recovery_settle_time:
            return self._decision(status="RECOVERY_SETTLING")

        if not self.planner.collision_filter.is_state_valid(
            self.current_q
        ):
            self.fault = True
            return self._decision(status="RECOVERY_UNSAFE")

        self.planning_required = False

        try:
            goal = self.planner.select_goal(
                self.target_base,
                self.goal_tolerance,
                self.target_clearance,
            )

            if goal is None or not self.planner.plan(
                self.current_q.copy(),
                goal,
            ):
                self.fault = True
                return self._decision(status="RECOVERY_NO_PATH")

            path = self.planner.prepare_execution_path(
                self.planner.current_path
            )

            old = self.failed_execution_path

            if (
                old is not None
                and len(old) == len(path)
                and np.allclose(
                    path[1:],
                    old[1:],
                    atol=0.02,
                    rtol=0.0,
                )
            ):
                self.fault = True
                return self._decision(status="RECOVERY_SAME_PATH")

            self.goal_q = np.asarray(goal, dtype=float).copy()
            self.current_path = self.planner.current_path.copy()
            self.path_sent = False
            self.stop_owned = True

            return self._send(
                "RELEASE",
                "WAITING_FOR_RELEASE",
            )

        except Exception:
            self.fault = True
            return self._decision(
                status="RECOVERY_PLANNING_ERROR"
            )

    def _prepare_safe_recovery(self):
        # Contact retreat has not been validated for human obstacles.
        if self.human_avoidance_enabled:
            return False

        if self.safe_recovery_attempted or not self.execute_path:
            return False

        self.safe_recovery_attempted = True

        find_path = getattr(
            self.planner,
            "find_safe_recovery_path",
            None,
        )

        if find_path is None:
            return False

        path = find_path(self.current_q.copy())

        if path is None:
            return False

        path = np.asarray(path, dtype=float)

        if (
            path.ndim != 2
            or path.shape[1] != 5
            or len(path) < 2
            or not np.isfinite(path).all()
            or np.max(np.abs(path[0] - self.current_q)) > 0.01
        ):
            return False

        checker = self.planner.collision_filter
        revision = checker.environment_revision
        state = checker.recovery_contact_state(path[0])

        if state is None:
            return False

        for start, goal in zip(path[:-1], path[1:]):
            if not checker.is_recovery_transition_valid(
                start,
                goal,
                state,
            ):
                return False

            state = checker.recovery_contact_state(goal)

            if state is None:
                return False

        if (
            checker.environment_revision != revision
            or not checker.is_state_valid(path[-1])
        ):
            return False

        self.current_path = path.copy()
        self.goal_q = None
        self.path_sent = False

        self.safe_recovery_active = True
        self.safe_recovery_revision = revision
        self.safe_recovery_initial_state = (
            checker.recovery_contact_state(path[0])
        )

        self.recovery_blocked = False
        return True

    def _prepare_safe_position(self):
        """Find a collision-free staging move from a valid joint state."""
        if not self.execute_path or not self.feedback_ready():
            return False

        checker = self.planner.collision_filter
        revision = checker.environment_revision
        if self.safe_position_attempted_revision == revision:
            return False
        self.safe_position_attempted_revision = revision

        try:
            if not checker.is_state_valid(self.current_q):
                # A contact retreat must be validated separately.
                return False

            candidates = np.asarray(self.planner.joint_q, dtype=float)
            if candidates.ndim != 2 or candidates.shape[1] != 5:
                return False

            distances = np.max(np.abs(candidates - self.current_q), axis=1)
            max_delta = float(self.planner.max_joint_delta)
            indices = np.where((distances > 0.05) & (distances <= max_delta))[0]
            indices = indices[np.argsort(distances[indices])]

            for index in indices[:40]:
                goal = candidates[index]
                if not checker.is_state_valid(goal):
                    continue
                if not checker.is_edge_valid(self.current_q, goal):
                    continue
                path = np.vstack((self.current_q.copy(), goal.copy()))
                prepared = self.planner.prepare_execution_path(path)
                if not checker.is_path_valid(prepared):
                    continue
                if checker.environment_revision != revision:
                    return False
                self.current_path = path
                self.goal_q = goal.copy()
                self.path_sent = False
                self.planning_required = False
                self.safe_position_active = True
                self.recovery_blocked = False
                return True
        except Exception:
            return False
        return False

    def invalidate_path(self):
        self.goal_q = None
        self.current_path = None
        self.planning_required = True
        self.path_sent = False

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

    def _decision(self, action="NONE", status=None, path=None):
        if status is not None:
            self.status = status

        return PlanningDecision(
            action=action,
            status=self.status,
            path=path,
        )

    def _send(self, action, status, path=None):
        self.pending_action = action
        self.pending_since = time.monotonic()

        if action == "STOP":
            self.stop_owned = True

        elif action == "EXECUTE":
            self.failure_reason = "NONE"
            self.path_sent = True

        return self._decision(
            action=action,
            status=status,
            path=path,
        )

    def _stop_required(self):
        return (
            self.motion_active
            or self.pending_action == "EXECUTE"
            or self.motion_status == "EXECUTING"
        )

    def step(self):
        now = time.monotonic()

        if self.pending_action == "STOP":
            if now - self.pending_since > self.command_timeout:
                self.fault = True
                return self._decision(status="STOP_TIMEOUT")

            return self._decision(status="WAITING_FOR_STOP")

        if self.human_avoidance_enabled:
            problem = self._human_guard()

            if problem is not None:
                if (
                    self._stop_required()
                    or self.pending_action == "RELEASE"
                ):
                    return self._send("STOP", problem)

                return self._decision(status=problem)

        if self.awaiting_failure_reason_since is not None:
            if now - self.awaiting_failure_reason_since < 0.5:
                return self._decision(
                    status="WAITING_FOR_FAILURE_REASON"
                )

            self._classify_motion_error()

        if self.fault:
            return self._decision(status="MOTION_ERROR")

        if self.recovery_pending and self.pending_action is None:
            return self._recovery_step(now)

        if self.pending_action in ("EXECUTE", "RELEASE"):
            if now - self.pending_since > self.command_timeout:
                self.fault = True

                return self._send(
                    "STOP",
                    "MOTION_RESPONSE_TIMEOUT",
                )

        if not self.feedback_ready():
            if self._stop_required():
                return self._send(
                    "STOP",
                    "WAITING_FOR_FEEDBACK",
                )

            return self._decision(
                status="WAITING_FOR_FEEDBACK"
            )

        if not self.obstacle_data_ready():
            if self._stop_required():
                return self._send(
                    "STOP",
                    "WAITING_FOR_OBSTACLES",
                )

            return self._decision(
                status="WAITING_FOR_OBSTACLES"
            )

        if self.safe_recovery_active and self._stop_required():
            checker = self.planner.collision_filter
            state = checker.recovery_contact_state(self.current_q)
            baseline = self.safe_recovery_initial_state

            unsafe = (
                state is None
                or baseline is None
                or checker.environment_revision
                != self.safe_recovery_revision
            )

            if not unsafe:
                for key, distance in state["pairs"].items():
                    if (
                        key not in baseline["pairs"]
                        or distance < baseline["pairs"][key] - 0.002
                    ):
                        unsafe = True
                        break

            if unsafe:
                self.safe_recovery_active = False
                self.fault = True

                return self._send("STOP", "RECOVERY_UNSAFE")

        if self.target_base is None:
            if self._stop_required():
                return self._send(
                    "STOP",
                    "WAITING_FOR_TARGET",
                )

            return self._decision(status="WAITING_FOR_TARGET")

        if self.resume_after_fault:
            try:
                state_safe = (
                    self.planner.collision_filter.is_state_valid(
                        self.current_q
                    )
                )
            except Exception:
                state_safe = False

            if not state_safe:
                self.fault = True
                return self._decision(status="RECOVERY_UNSAFE")

        if self.planning_required:
            if self._stop_required():
                return self._send(
                    "STOP",
                    "WAITING_FOR_STOP",
                )

            if self.pending_action == "RELEASE":
                return self._decision(
                    status="WAITING_FOR_RELEASE"
                )

            self.planning_required = False

            try:
                goal = self.planner.select_goal(
                    self.target_base,
                    self.goal_tolerance,
                    self.target_clearance,
                )

                if goal is None:
                    return self._decision(
                        status="GOAL_UNREACHABLE"
                    )

                if not self.planner.plan(
                    self.current_q.copy(),
                    goal,
                ):
                    self.human_retry_count += 1
                    self.human_replan_blocked = True
                    self.human_retry_revision = (
                        self.planner.collision_filter.environment_revision
                    )
                    if (
                        self.planner.last_plan_failure
                        == "START_INVALID"
                    ):
                        if self._prepare_safe_recovery():
                            return self._decision(
                                status="SAFE_RECOVERY_READY"
                            )

                        self.recovery_blocked = True
                        return self._decision(
                            status="RECOVERY_BLOCKED"
                        )

                    if self.human_retry_count >= 2 and self._prepare_safe_position():
                        return self._decision(status="SAFE_POSITION_READY")
                    return self._decision(status="NO_PATH")

                self.human_retry_count = 0
                self.goal_q = np.asarray(
                    goal,
                    dtype=float,
                ).copy()

                self.current_path = (
                    self.planner.current_path.copy()
                )

                self.path_sent = False
                self.human_replan_blocked = False
                return self._decision(status="PATH_READY")

            except Exception:
                self.current_path = None
                self.goal_q = None

                return self._decision(status="PLANNING_ERROR")

        if self.current_path is None:
            if self.recovery_blocked:
                return self._decision(
                    status="RECOVERY_BLOCKED"
                )

            return self._decision()

        if not self.execute_path:
            return self._decision(status="PATH_READY")

        if self.path_sent:
            if self.pending_action == "EXECUTE":
                return self._decision(
                    status="TRAJECTORY_SENT"
                )

            if self.motion_active:
                return self._decision(status="EXECUTING")

            if self.motion_status == "COMPLETED":
                return self._decision(status="COMPLETED")

            return self._decision(
                status="WAITING_FOR_MOTION"
            )

        if self.pending_action == "RELEASE":
            return self._decision(
                status="WAITING_FOR_RELEASE"
            )

        if (
            self.resume_after_fault
            and self.motion_status == "CONTROL_ERROR"
        ):
            return self._send(
                "RELEASE",
                "WAITING_FOR_RELEASE",
            )

        if self.motion_status == "STOPPED":
            if self.stop_owned:
                return self._send(
                    "RELEASE",
                    "WAITING_FOR_RELEASE",
                )

            return self._decision(status="STOPPED")

        if self.motion_status not in ("IDLE", "COMPLETED"):
            return self._decision(
                status="WAITING_FOR_MOTION"
            )

        start_error = float(
            np.max(
                np.abs(
                    self.current_q - self.current_path[0]
                )
            )
        )

        if start_error > 0.05:
            self.invalidate_path()
            return self._decision(status="REPLANNING")

        try:
            path = self.current_path.copy()
            path[0] = self.current_q.copy()

            if self.safe_recovery_active:
                checker = self.planner.collision_filter

                if (
                    start_error > 0.01
                    or checker.environment_revision
                    != self.safe_recovery_revision
                    or checker.recovery_contact_state(
                        self.current_q
                    ) is None
                    or not checker.is_recovery_transition_valid(
                        self.current_q,
                        path[1],
                    )
                ):
                    raise ValueError(
                        "Recovery path is no longer safe"
                    )

                execution_path = path

            else:
                execution_path = (
                    self.planner.prepare_execution_path(path)
                )

        except Exception:
            self.current_path = None
            self.planning_required = False
            self.safe_recovery_active = False

            return self._decision(
                status="TRAJECTORY_ERROR"
            )

        if self.human_avoidance_enabled:
            checker = self.planner.collision_filter

            if (
                (
                    not checker.human_data_ready()
                    and not self.simulation_only_ignore_unknown
                )
                or not checker.is_path_valid(execution_path)
            ):
                self.invalidate_path()
                self.human_replan_blocked = True

                return self._decision(
                    status="HUMAN_PATH_BLOCKED"
                )

        self.last_execution_path = execution_path.copy()

        return self._send(
            "EXECUTE",
            (
                "SAFE_RECOVERY_SENT"
                if self.safe_recovery_active
                else "TRAJECTORY_SENT"
            ),
            execution_path,
        )