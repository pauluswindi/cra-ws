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


def nearest_human_collision(robot_geometries, capsules, safety_margin, include_pairs=False):
    """Return the closest robot-geometry/capsule pair.

    robot_geometries yields (geom_name, center_mujoco_base, bounding_radius).
    capsules contains parsed (body_id, segment_id), start, end, radius in
    robot base_link. Values and radii are in meters.

    Bounding spheres are intentionally retained for this first fix. The
    returned radii and distance make their conservatism visible in diagnostics.
    """
    geometries = list(robot_geometries)
    if not geometries or not capsules:
        return {"status": "UNKNOWN", "clearance": None,
                "reason": "NO_GEOMETRY"}

    try:
        centers = np.asarray([item[1] for item in geometries], dtype=float)
        radii = np.asarray([item[2] for item in geometries], dtype=float)
    except (TypeError, ValueError, OverflowError):
        return {"status": "UNKNOWN", "clearance": None,
                "reason": "INVALID_ROBOT_GEOMETRY"}
    if (
        centers.shape != (len(geometries), 3)
        or radii.shape != (len(geometries),)
        or not np.isfinite(centers).all() or not np.isfinite(radii).all()
        or np.any(radii < 0.)
    ):
        return {"status": "UNKNOWN", "clearance": None,
                "reason": "INVALID_ROBOT_GEOMETRY"}

    # Evaluate every pair in one NumPy batch. The geometry and the distance
    # formula are unchanged; this avoids dozens of small Python/NumPy calls
    # for each configuration sampled during graph search or margin retreat.
    centers = centers @ FrameTransform.ROBOT_FROM_MUJOCO_BASE.T
    starts = np.asarray([item[1] for item in capsules], dtype=float)
    axes = np.asarray([item[2] for item in capsules], dtype=float) - starts
    human_radii = np.asarray([item[3] for item in capsules], dtype=float)
    length_sq = np.einsum('ij,ij->i', axes, axes)
    offsets = centers[:, None, :] - starts[None, :, :]
    projection = np.einsum('ijk,jk->ij', offsets, axes)
    alpha = np.divide(projection, length_sq, out=np.zeros_like(projection),
                      where=length_sq > 1e-12)
    np.clip(alpha, 0., 1., out=alpha)
    distances = np.linalg.norm(offsets - alpha[:, :, None] * axes, axis=2)
    clearances = distances - radii[:, None] - human_radii[None, :]
    geom_index, capsule_index = np.unravel_index(np.argmin(clearances), clearances.shape)
    key = capsules[capsule_index][0]
    nearest = {
        "clearance": float(clearances[geom_index, capsule_index]),
        "robot_geom": geometries[geom_index][0],
        "body_id": key[0], "segment_id": key[1],
        "center_distance": float(distances[geom_index, capsule_index]),
        "robot_radius": float(radii[geom_index]),
        "human_radius": float(human_radii[capsule_index]),
        "safety_margin": float(safety_margin),
    }
    nearest["status"] = (
        "NEAR" if nearest["clearance"] <= safety_margin else "FAR"
    )
    if include_pairs:
        nearest["pairs"] = {
            (i, item[0][0], item[0][1]): float(clearances[i, j])
            for i in range(len(geometries)) for j, item in enumerate(capsules)
        }
    return nearest
