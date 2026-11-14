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


def mesh_local_box(model, geom_id):
    """Enclose every compiled mesh vertex in its geom-local frame.

    MuJoCo has already applied asset scale, mesh centering and rotation to
    mesh_vert. The box follows geom_xpos/geom_xmat, not the original STL pose.
    A convex box containing all vertices also contains every mesh triangle.
    """
    mesh = int(model.geom_dataid[geom_id])
    first, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
    vertices = np.asarray(model.mesh_vert[first:first + count], dtype=float)
    if count == 0 or not np.isfinite(vertices).all():
        raise ValueError("Invalid robot mesh vertices")
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    # Numerical enclosure padding, in meters; this is not the human margin.
    return (lower + upper) * .5, (upper - lower) * .5 + 1e-6


def segment_box_distances(starts, ends, half_extents):
    """Exact unsigned segment/AABB distances, batched over leading axes.

    Distance squared is piecewise quadratic in segment parameter t. Split
    [0, 1] at all six box-face crossings and minimize each quadratic, including
    its boundaries. This covers intersections, parallel and point segments.
    """
    starts, ends, half_extents = np.broadcast_arrays(
        *[np.asarray(value, dtype=float) for value in (starts, ends, half_extents)])
    delta = ends - starts
    crossings = [np.zeros(starts.shape[:-1] + (1,)), np.ones(starts.shape[:-1] + (1,))]
    for face in (-half_extents, half_extents):
        roots = np.divide(face - starts, delta, out=np.zeros_like(starts), where=delta != 0.)
        crossings.append(np.clip(roots, 0., 1.))
    bounds = np.sort(np.concatenate(crossings, axis=-1), axis=-1)
    left, right = bounds[..., :-1], bounds[..., 1:]
    middle = starts[..., None, :] + ((left + right) * .5)[..., None] * delta[..., None, :]
    outside = np.abs(middle) > half_extents[..., None, :]
    offset = np.where(outside, starts[..., None, :]
                      - np.sign(middle) * half_extents[..., None, :], 0.)
    slope = np.where(outside, delta[..., None, :], 0.)
    denominator = np.sum(slope * slope, axis=-1)
    parameter = np.divide(-np.sum(offset * slope, axis=-1), denominator,
                          out=left.copy(), where=denominator > 0.)
    parameter = np.clip(parameter, left, right)
    points = starts[..., None, :] + parameter[..., None] * delta[..., None, :]
    gaps = np.maximum(np.abs(points) - half_extents[..., None, :], 0.)
    return np.sqrt(np.min(np.sum(gaps * gaps, axis=-1), axis=-1))


def nearest_human_collision(robot_geometries, capsules, safety_margin, include_pairs=False,
                            robot_boxes=None):
    """Return the closest robot-geometry/capsule pair.

    robot_geometries yields (geom_name, center_mujoco_base, bounding_radius).
    capsules contains parsed (body_id, segment_id), start, end, radius in
    robot base_link. Values and radii are in meters.

    Optional robot_boxes aligns with robot_geometries and contains None or
    (center_mujoco_base, rotation_mujoco_base, half_extents). Both sphere and
    box enclose the mesh: the greater of their clearance lower bounds remains
    conservative for the mesh. Existing sphere-only callers stay compatible.
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
    sphere_clearances = distances - radii[:, None] - human_radii[None, :]
    clearances = sphere_clearances.copy()
    box_clearances = np.full_like(clearances, np.nan)
    # A sphere-clear pair is already certified beyond the margin. The exact
    # box distance is only needed for potential proximity, or for consistent
    # all-pair clearance values used by the monotonic retreat certificate.
    if robot_boxes is not None and (include_pairs or np.any(sphere_clearances <= safety_margin)):
        try:
            if len(robot_boxes) != len(geometries):
                raise ValueError("Mismatched robot boxes")
            indices = [i for i, box in enumerate(robot_boxes) if box is not None]
            if indices:
                box_centers, rotations, halves = (
                    np.asarray([robot_boxes[i][j] for i in indices], dtype=float)
                    for j in range(3)
                )
                count = len(indices)
                if (
                    box_centers.shape != (count, 3) or rotations.shape != (count, 3, 3)
                    or halves.shape != (count, 3) or not np.isfinite(box_centers).all()
                    or not np.isfinite(rotations).all() or not np.isfinite(halves).all()
                    or np.any(halves <= 0.)
                    or not np.allclose(np.swapaxes(rotations, -1, -2) @ rotations,
                                       np.eye(3), atol=1e-9, rtol=0.)
                ):
                    raise ValueError("Invalid robot boxes")
                # Convert CRA capsule endpoints back to MuJoCo base, then into
                # each box's local axes. Distances are invariant under rotations.
                capsule_starts = starts @ FrameTransform.ROBOT_FROM_MUJOCO_BASE
                capsule_ends = (starts + axes) @ FrameTransform.ROBOT_FROM_MUJOCO_BASE
                local_starts = np.einsum('gci,gij->gcj',
                                         capsule_starts[None] - box_centers[:, None], rotations)
                local_ends = np.einsum('gci,gij->gcj',
                                       capsule_ends[None] - box_centers[:, None], rotations)
                box_clearances[indices] = segment_box_distances(
                    local_starts, local_ends, halves[:, None]) - human_radii[None]
                clearances[indices] = np.maximum(clearances[indices], box_clearances[indices])
        except (TypeError, ValueError, IndexError, OverflowError):
            return {"status": "UNKNOWN", "clearance": None,
                    "reason": "INVALID_ROBOT_BOX"}
    if not np.isfinite(clearances).all():
        return {"status": "UNKNOWN", "clearance": None,
                "reason": "INVALID_GEOMETRY_DISTANCE"}
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
        "geometry_model": "SPHERE_AND_MESH_BOX" if np.isfinite(
            box_clearances[geom_index, capsule_index]) else "SPHERE",
        "sphere_clearance": float(sphere_clearances[geom_index, capsule_index]),
        "box_clearance": (float(box_clearances[geom_index, capsule_index]) if np.isfinite(
            box_clearances[geom_index, capsule_index]) else None),
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
