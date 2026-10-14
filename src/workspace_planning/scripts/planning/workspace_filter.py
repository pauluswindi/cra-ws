#!/usr/bin/env python3
"""Filter GNG workspace nodes and edges using target and obstacle geometry."""

from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


@dataclass(frozen=True)
class TargetGeometry:
    """Target mesh bounds expressed relative to the target body origin."""

    initial_position: np.ndarray
    center_offset: np.ndarray
    size: np.ndarray


def load_target_geometry(model_path, target_body_name="target", base_body_name="base_link"):
    """Load target geometry from MuJoCo XML and return conservative mesh bounds.

    The target's geometry comes from XML. Its position can subsequently be updated
    by the current target-position topic while preserving this geometry size.
    """
    try:
        import mujoco
    except ImportError as error:
        raise RuntimeError(
            "The MuJoCo Python package is required to load the target geometry "
            "from the robot XML model."
        ) from error

    model_path = Path(model_path).expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"MuJoCo model was not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    target_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, target_body_name
    )
    base_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, base_body_name
    )

    if target_body_id < 0:
        raise ValueError(f'Body "{target_body_name}" was not found in the XML model.')
    if base_body_id < 0:
        raise ValueError(f'Body "{base_body_name}" was not found in the XML model.')

    geom_ids = np.flatnonzero(model.geom_bodyid == target_body_id)
    if len(geom_ids) == 0:
        raise ValueError(f'Body "{target_body_name}" does not contain any geometry.')

    points_world = []
    for geom_id in geom_ids:
        points_local = _get_geom_bounding_vertices(model, int(geom_id), mujoco)
        if len(points_local) == 0:
            continue

        rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        position = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        points_world.append(points_local @ rotation.T + position)

    if not points_world:
        raise ValueError(
            f'Could not read a supported geometry from body "{target_body_name}".'
        )

    points_world = np.vstack(points_world)
    base_origin_world = np.asarray(data.xpos[base_body_id], dtype=np.float64)
    base_rotation_world = np.asarray(
        data.xmat[base_body_id], dtype=np.float64
    ).reshape(3, 3)

    points_mujoco_base = (points_world - base_origin_world) @ base_rotation_world
    target_points = _transform_mujoco_positions(points_mujoco_base)

    target_origin_mujoco_base = (
        np.asarray(data.xpos[target_body_id], dtype=np.float64) - base_origin_world
    ) @ base_rotation_world
    target_origin = _transform_mujoco_positions(target_origin_mujoco_base.reshape(1, 3))[0]

    bounds_min = np.min(target_points, axis=0)
    bounds_max = np.max(target_points, axis=0)
    bounds_center = 0.5 * (bounds_min + bounds_max)
    size = bounds_max - bounds_min

    if np.any(size <= 0.0):
        raise ValueError(
            f'Target body "{target_body_name}" has invalid or zero-sized bounds: {size}.'
        )

    return TargetGeometry(
        initial_position=target_origin,
        center_offset=bounds_center - target_origin,
        size=size,
    )


def _transform_mujoco_positions(positions):
    """Use the project's existing MuJoCo-base to robot-base transform."""
    try:
        from workspace.frame_transform import FrameTransform
    except ImportError as error:
        raise RuntimeError(
            "Could not import workspace.frame_transform.FrameTransform."
        ) from error

    positions = np.asarray(positions, dtype=np.float64)
    if positions.ndim == 1:
        return FrameTransform.mujoco_base_to_robot_base(positions)
    return FrameTransform.transform_positions(positions)


def _get_geom_bounding_vertices(model, geom_id, mujoco):
    """Return local vertices for a mesh or conservative bounds for a primitive."""
    geom_type = int(model.geom_type[geom_id])
    geom_size = np.asarray(model.geom_size[geom_id], dtype=np.float64)

    if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[geom_id])
        vertex_start = int(model.mesh_vertadr[mesh_id])
        vertex_count = int(model.mesh_vertnum[mesh_id])
        if vertex_count <= 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(
            model.mesh_vert[vertex_start:vertex_start + vertex_count],
            dtype=np.float64,
        ).copy()

    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        half_size = geom_size[:3]
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        half_size = np.repeat(geom_size[0], 3)
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        half_size = geom_size[:3]
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        half_size = np.array([geom_size[0], geom_size[0], geom_size[1]])
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        half_size = np.array(
            [geom_size[0], geom_size[0], geom_size[0] + geom_size[1]]
        )
    else:
        return np.empty((0, 3), dtype=np.float64)

    return _box_vertices(half_size)


def _box_vertices(half_size):
    hx, hy, hz = np.asarray(half_size, dtype=np.float64)
    return np.array(
        [
            [-hx, -hy, -hz], [-hx, -hy, hz],
            [-hx, hy, -hz], [-hx, hy, hz],
            [hx, -hy, -hz], [hx, -hy, hz],
            [hx, hy, -hz], [hx, hy, hz],
        ],
        dtype=np.float64,
    )


class WorkspaceFilter:
    """Generate valid-node and valid-edge masks without modifying the GNG graph."""

    SUPPORTED_SHAPES = {"box", "sphere", "cylinder"}

    def __init__(self, positions, edges, safety_margin=0.01):
        self.positions = np.asarray(positions, dtype=np.float64)
        self.edges = np.asarray(edges, dtype=np.int64)
        self.safety_margin = float(safety_margin)

        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError("Positions must have shape (N, 3).")
        if self.edges.ndim != 2 or self.edges.shape[1] != 2:
            raise ValueError("Edges must have shape (M, 2).")
        if not np.all(np.isfinite(self.positions)):
            raise ValueError("Workspace positions must contain finite values only.")
        if not np.isfinite(self.safety_margin) or self.safety_margin < 0.0:
            raise ValueError("Safety margin must be a finite, non-negative number.")

        self.target_box_center = None
        self.target_box_size = None
        self.obstacle = None
        self.valid_nodes = np.ones(len(self.positions), dtype=bool)
        self.valid_edges = np.ones(len(self.edges), dtype=bool)

        if len(self.edges):
            if np.any(self.edges < 0) or np.any(self.edges >= len(self.positions)):
                raise ValueError("An edge references a node index outside the workspace graph.")

    def set_target_box(self, center, size):
        center = self._vector(center, "Target center")
        size = self._positive_size(size, "Target size")
        unchanged = (
            self.target_box_center is not None
            and self.target_box_size is not None
            and np.allclose(center, self.target_box_center, atol=1e-6)
            and np.allclose(size, self.target_box_size, atol=1e-6)
        )
        self.target_box_center = center
        self.target_box_size = size
        return not unchanged

    def set_obstacle(self, shape_type, center, size, quaternion=(0.0, 0.0, 0.0, 1.0)):
        shape_type = str(shape_type).lower()
        if shape_type not in self.SUPPORTED_SHAPES:
            raise ValueError(
                f"Unsupported obstacle shape '{shape_type}'. "
                f"Choose one of: {', '.join(sorted(self.SUPPORTED_SHAPES))}."
            )

        candidate = {
            "shape": shape_type,
            "center": self._vector(center, "Obstacle center"),
            "size": self._positive_size(size, "Obstacle size"),
            "rotation": self._quaternion_to_rotation(quaternion),
        }
        unchanged = (
            self.obstacle is not None
            and self.obstacle["shape"] == candidate["shape"]
            and np.allclose(self.obstacle["center"], candidate["center"], atol=1e-6)
            and np.allclose(self.obstacle["size"], candidate["size"], atol=1e-6)
            and np.allclose(self.obstacle["rotation"], candidate["rotation"], atol=1e-6)
        )
        self.obstacle = candidate
        return not unchanged

    def clear_obstacle(self):
        had_obstacle = self.obstacle is not None
        self.obstacle = None
        return had_obstacle

    def update(self):
        """Recalculate masks after target or obstacle geometry changes."""
        self.valid_nodes = np.array(
            [not self.is_point_blocked(position) for position in self.positions],
            dtype=bool,
        )

        valid_edges = np.ones(len(self.edges), dtype=bool)
        for edge_index, (node_a, node_b) in enumerate(self.edges):
            node_a = int(node_a)
            node_b = int(node_b)

            if not self.valid_nodes[node_a] or not self.valid_nodes[node_b]:
                valid_edges[edge_index] = False
                continue

            if self.is_segment_blocked(self.positions[node_a], self.positions[node_b]):
                valid_edges[edge_index] = False

        self.valid_edges = valid_edges
        return self.valid_nodes.copy(), self.valid_edges.copy()

    def is_point_blocked(self, point, include_target=True, include_obstacle=True):
        point = self._vector(point, "Point")

        if include_target and self.target_box_center is not None:
            half_size = self.target_box_size * 0.5 + self.safety_margin
            local_point = point - self.target_box_center
            if np.all(np.abs(local_point) <= half_size):
                return True

        if include_obstacle and self.obstacle is not None:
            local_point = self._to_obstacle_local(point)
            shape = self.obstacle["shape"]
            half_size = self.obstacle["size"] * 0.5
            margin = self.safety_margin

            if shape == "box":
                return bool(np.all(np.abs(local_point) <= half_size + margin))

            if shape == "sphere":
                radii = half_size + margin
                return bool(np.sum((local_point / radii) ** 2) <= 1.0)

            if shape == "cylinder":
                radii = half_size[:2] + margin
                radial_value = np.sum((local_point[:2] / radii) ** 2)
                return bool(
                    radial_value <= 1.0
                    and abs(local_point[2]) <= half_size[2] + margin
                )

        return False

    def is_segment_blocked(
        self,
        start,
        end,
        include_target=True,
        include_obstacle=True,
    ):
        start = self._vector(start, "Segment start")
        end = self._vector(end, "Segment end")

        if include_target and self.target_box_center is not None:
            local_start = start - self.target_box_center
            local_end = end - self.target_box_center
            half_size = self.target_box_size * 0.5 + self.safety_margin
            if self._segment_intersects_box(local_start, local_end, half_size):
                return True

        if include_obstacle and self.obstacle is not None:
            local_start = self._to_obstacle_local(start)
            local_end = self._to_obstacle_local(end)
            shape = self.obstacle["shape"]
            half_size = self.obstacle["size"] * 0.5
            margin = self.safety_margin

            if shape == "box":
                return self._segment_intersects_box(
                    local_start, local_end, half_size + margin
                )

            if shape == "sphere":
                radii = half_size + margin
                scaled_start = local_start / radii
                scaled_end = local_end / radii
                return self._segment_intersects_unit_ellipsoid(
                    scaled_start, scaled_end
                )

            if shape == "cylinder":
                radii = half_size[:2] + margin
                half_height = half_size[2] + margin
                return self._segment_intersects_cylinder(
                    local_start, local_end, radii, half_height
                )

        return False

    def _to_obstacle_local(self, point):
        difference = point - self.obstacle["center"]
        return self.obstacle["rotation"].T @ difference

    @staticmethod
    def _segment_intersects_box(start, end, half_size):
        direction = end - start
        lower = -np.asarray(half_size, dtype=np.float64)
        upper = np.asarray(half_size, dtype=np.float64)
        t_min = 0.0
        t_max = 1.0

        for axis in range(3):
            if abs(direction[axis]) < 1e-12:
                if start[axis] < lower[axis] or start[axis] > upper[axis]:
                    return False
                continue

            first = (lower[axis] - start[axis]) / direction[axis]
            second = (upper[axis] - start[axis]) / direction[axis]
            if first > second:
                first, second = second, first

            t_min = max(t_min, first)
            t_max = min(t_max, second)
            if t_min > t_max:
                return False

        return True

    @staticmethod
    def _segment_intersects_unit_ellipsoid(start, end):
        direction = end - start
        denominator = float(np.dot(direction, direction))
        if denominator < 1e-12:
            return bool(np.dot(start, start) <= 1.0)

        closest_t = float(np.clip(-np.dot(start, direction) / denominator, 0.0, 1.0))
        closest = start + closest_t * direction
        return bool(np.dot(closest, closest) <= 1.0)

    @classmethod
    def _segment_intersects_cylinder(cls, start, end, radii, half_height):
        start_xy = start[:2] / radii
        end_xy = end[:2] / radii
        radial_interval = cls._quadratic_inside_interval(start_xy, end_xy - start_xy)
        if radial_interval is None:
            return False

        z_interval = cls._linear_inside_interval(
            start[2], end[2] - start[2], -half_height, half_height
        )
        if z_interval is None:
            return False

        return max(radial_interval[0], z_interval[0]) <= min(
            radial_interval[1], z_interval[1]
        )

    @staticmethod
    def _quadratic_inside_interval(start, direction):
        a = float(np.dot(direction, direction))
        b = 2.0 * float(np.dot(start, direction))
        c = float(np.dot(start, start)) - 1.0

        if a < 1e-12:
            return (0.0, 1.0) if c <= 0.0 else None

        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return (0.0, 1.0) if c <= 0.0 else None

        root = np.sqrt(max(discriminant, 0.0))
        first = (-b - root) / (2.0 * a)
        second = (-b + root) / (2.0 * a)
        lower = max(0.0, min(first, second))
        upper = min(1.0, max(first, second))
        if lower > upper:
            return None
        return lower, upper

    @staticmethod
    def _linear_inside_interval(start, direction, lower_bound, upper_bound):
        if abs(direction) < 1e-12:
            if lower_bound <= start <= upper_bound:
                return 0.0, 1.0
            return None

        first = (lower_bound - start) / direction
        second = (upper_bound - start) / direction
        lower = max(0.0, min(first, second))
        upper = min(1.0, max(first, second))
        if lower > upper:
            return None
        return lower, upper

    @staticmethod
    def _vector(value, name):
        value = np.asarray(value, dtype=np.float64)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain three finite values.")
        return value.copy()

    @classmethod
    def _positive_size(cls, value, name):
        value = cls._vector(value, name)
        if np.any(value <= 0.0):
            raise ValueError(f"{name} dimensions must be greater than zero.")
        return value

    @staticmethod
    def _quaternion_to_rotation(quaternion):
        quaternion = np.asarray(quaternion, dtype=np.float64)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("Obstacle quaternion must contain four finite values.")

        norm = np.linalg.norm(quaternion)
        if norm < 1e-12:
            quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            quaternion = quaternion / norm

        x, y, z, w = quaternion
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
