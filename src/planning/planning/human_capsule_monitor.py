#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class HumanCapsule:
    body_id: int
    segment_id: int
    start: tuple
    end: tuple
    radius: float


@dataclass(frozen=True)
class HumanSnapshot:
    status: str
    reason: str
    capsules: tuple

    @property
    def human_count(self):
        return len({capsule.body_id for capsule in self.capsules})


class HumanCapsuleMonitor:
    def __init__(
        self,
        frame_id="base_link",
        data_timeout=0.5,
        future_tolerance=0.1,
    ):
        self.frame_id = str(frame_id)
        self.data_timeout = float(data_timeout)
        self.future_tolerance = float(future_tolerance)

        if (
            not math.isfinite(self.data_timeout)
            or self.data_timeout <= 0.0
            or not math.isfinite(self.future_tolerance)
            or self.future_tolerance < 0.0
        ):
            raise ValueError(
                "Invalid Human Capsule monitor configuration"
            )

        self.status = "UNKNOWN"
        self.reason = "NO_DATA"

        self.capsules = ()
        self.stamp_ns = None
        self.received_time = None

    @staticmethod
    def _point_tuple(point):
        coordinates = (
            float(point.x),
            float(point.y),
            float(point.z),
        )

        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError("Non-finite capsule coordinates")

        return coordinates

    @staticmethod
    def _stamp_nanoseconds(stamp):
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)

        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError("Invalid message timestamp")

        result = seconds * 1_000_000_000 + nanoseconds

        if result <= 0:
            raise ValueError("Missing message timestamp")

        return result

    def _set_unknown(self, reason):
        self.status = "UNKNOWN"
        self.reason = reason
        self.capsules = ()
        self.stamp_ns = None
        self.received_time = None

    def update(self, message, now_ros_ns):
        """Validate and store the latest human geometry."""

        if not message.valid:
            self._set_unknown("VISION_INVALID")
            return False

        if message.header.frame_id != self.frame_id:
            self._set_unknown("FRAME_MISMATCH")
            return False

        try:
            stamp_ns = self._stamp_nanoseconds(
                message.header.stamp
            )

            now_ros_ns = int(now_ros_ns)

            if now_ros_ns <= 0:
                raise ValueError("ROS clock is not initialized")

            # Ignore delayed observations older than the current data.
            if (
                self.stamp_ns is not None
                and stamp_ns <= self.stamp_ns
            ):
                return True

            age = (now_ros_ns - stamp_ns) / 1e9

            if age > self.data_timeout:
                self._set_unknown("STALE_MESSAGE")
                return False

            if age < -self.future_tolerance:
                self._set_unknown("FUTURE_TIMESTAMP")
                return False

            capsules = []
            identifiers = set()

            for item in message.capsules:
                body_id = int(item.body_id)
                segment_id = int(item.segment_id)

                if body_id < 0 or not 0 <= segment_id <= 7:
                    raise ValueError("Invalid capsule identifier")

                identifier = (body_id, segment_id)

                if identifier in identifiers:
                    raise ValueError("Duplicate capsule segment")

                identifiers.add(identifier)

                start = self._point_tuple(item.start)
                end = self._point_tuple(item.end)
                radius = float(item.radius)

                if not math.isfinite(radius) or radius <= 0.0:
                    raise ValueError("Invalid capsule radius")

                capsules.append(
                    HumanCapsule(
                        body_id=body_id,
                        segment_id=segment_id,
                        start=start,
                        end=end,
                        radius=radius,
                    )
                )

        except (AttributeError, TypeError, ValueError, OverflowError):
            self._set_unknown("INVALID_GEOMETRY")
            return False

        self.capsules = tuple(capsules)
        self.stamp_ns = stamp_ns
        self.received_time = time.monotonic()

        if self.capsules:
            self.status = "OBSERVED"
            self.reason = "CAPSULES_PRESENT"
        else:
            self.status = "EMPTY"
            self.reason = "NO_ACTIVE_CAPSULES"

        return True

    def snapshot(self, now_ros_ns):
        """Return the latest observation if it is still fresh."""

        if self.received_time is None or self.stamp_ns is None:
            return HumanSnapshot(
                status="UNKNOWN",
                reason=self.reason,
                capsules=(),
            )

        elapsed = time.monotonic() - self.received_time

        if elapsed > self.data_timeout:
            return HumanSnapshot(
                status="UNKNOWN",
                reason="MESSAGE_TIMEOUT",
                capsules=(),
            )

        now_ros_ns = int(now_ros_ns)

        if now_ros_ns <= 0:
            return HumanSnapshot(
                status="UNKNOWN",
                reason="INVALID_ROS_TIME",
                capsules=(),
            )

        age = (now_ros_ns - self.stamp_ns) / 1e9

        if age > self.data_timeout:
            return HumanSnapshot(
                status="UNKNOWN",
                reason="STALE_MESSAGE",
                capsules=(),
            )

        if age < -self.future_tolerance:
            return HumanSnapshot(
                status="UNKNOWN",
                reason="CLOCK_MISMATCH",
                capsules=(),
            )

        return HumanSnapshot(
            status=self.status,
            reason=self.reason,
            capsules=self.capsules,
        )