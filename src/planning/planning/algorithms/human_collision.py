"""Human capsule validation and distance checks for CRA Planning.

Input human capsule coordinates are expected in the CRA robot base_link frame.
Robot geometry centers are supplied in the local MuJoCo base frame, then
converted using the same FrameTransform employed by the Planning node.
"""

import numpy as np

from simulation.frame_transform import FrameTransform


def parse_human_snapshot(snapshot):
    """Convert a validated HumanSnapshot to immutable collision geometry.

    Non-observed and malformed snapshots remain UNKNOWN (fail-closed).
    The input is not a raw ROS message.
    """
    if getattr(snapshot, "status", "UNKNOWN") != "OBSERVED":
        return "UNKNOWN", ()

    try:
        parsed = []
        identifiers = set()

        for item in snapshot.capsules:
            key = (int(item.body_id), int(item.segment_id))
            if key in identifiers or key[0] < 0 or not 0 <= key[1] <= 7:
                raise ValueError("Invalid human capsule identifier")
            identifiers.add(key)

            start = np.asarray(item.start, dtype=float)
            end = np.asarray(item.end, dtype=float)
            radius = float(item.radius)

            if (
                start.shape != (3,)
                or end.shape != (3,)
                or not np.isfinite(start).all()
                or not np.isfinite(end).all()
                or not np.isfinite(radius)
                or radius <= 0.0
            ):
                raise ValueError("Invalid human capsule geometry")

            parsed.append((key, tuple(start), tuple(end), radius))

        if not parsed:
            raise ValueError("No human capsules")

        return "OBSERVED", tuple(sorted(parsed))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return "UNKNOWN", ()


def point_segment_distance(point, start, end):
    """Shortest distance from a point to a finite 3D line segment."""
    axis = end - start
    length_sq = float(axis @ axis)

    if length_sq <= 1e-12:
        return float(np.linalg.norm(point - start))

    alpha = np.clip(float((point - start) @ axis) / length_sq, 0.0, 1.0)
    return float(np.linalg.norm(point - (start + alpha * axis)))


def nearest_human_collision(robot_geometries, capsules, safety_margin):
    """Return the closest robot-geometry/capsule pair.

    robot_geometries yields (geom_name, center_mujoco_base, bounding_radius).
    capsules contains parsed (body_id, segment_id), start, end, radius in
    robot base_link. Values and radii are in meters.

    Bounding spheres are intentionally retained for this first fix. The
    returned radii and distance make their conservatism visible in diagnostics.
    """
    nearest = None
    rotation = FrameTransform.ROBOT_FROM_MUJOCO_BASE

    for name, center_mujoco, robot_radius in robot_geometries:
        robot_radius = float(robot_radius)
        center_mujoco = np.asarray(center_mujoco, dtype=float)

        if (
            center_mujoco.shape != (3,)
            or not np.isfinite(center_mujoco).all()
            or not np.isfinite(robot_radius)
            or robot_radius < 0.0
        ):
            return {"status": "UNKNOWN", "clearance": None,
                    "reason": "INVALID_ROBOT_GEOMETRY"}

        center = rotation @ center_mujoco

        for key, start, end, human_radius in capsules:
            distance = point_segment_distance(
                center,
                np.asarray(start, dtype=float),
                np.asarray(end, dtype=float),
            )
            clearance = distance - robot_radius - human_radius

            if nearest is None or clearance < nearest["clearance"]:
                nearest = {
                    "clearance": float(clearance),
                    "robot_geom": name,
                    "body_id": key[0],
                    "segment_id": key[1],
                    "center_distance": float(distance),
                    "robot_radius": robot_radius,
                    "human_radius": float(human_radius),
                    "safety_margin": float(safety_margin),
                }

    if nearest is None:
        return {"status": "UNKNOWN", "clearance": None,
                "reason": "NO_GEOMETRY"}

    nearest["status"] = (
        "NEAR" if nearest["clearance"] <= safety_margin else "FAR"
    )
    return nearest
