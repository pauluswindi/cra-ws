#!/usr/bin/env python3
"""Filter node dan edge GNG berdasarkan geometri target dan obstacle."""

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
    initial_position: np.ndarray
    center_offset: np.ndarray
    size: np.ndarray


def load_target_geometry(model_path, target_body_name="target", base_body_name="base_link"):
    # Membaca batas geometri target dari model MuJoCo
    import mujoco

    path = Path(model_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {path}")
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base_body_name)
    if target_id < 0 or base_id < 0:
        raise ValueError("Target or base body is absent from MuJoCo model")

    vertices_world = []
    for geom_id in np.flatnonzero(model.geom_bodyid == target_id):
        vertices = _get_geom_bounding_vertices(model, int(geom_id), mujoco)
        if vertices.size:
            rot = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
            pos = np.asarray(data.geom_xpos[geom_id])
            vertices_world.append(vertices @ rot.T + pos)
    if not vertices_world:
        raise ValueError("Target body has no supported geometry")

    base_pos = np.asarray(data.xpos[base_id])
    base_rot = np.asarray(data.xmat[base_id]).reshape(3, 3)
    vertices_base = (np.vstack(vertices_world) - base_pos) @ base_rot
    vertices_robot = _transform_mujoco_positions(vertices_base)
    origin_base = (np.asarray(data.xpos[target_id]) - base_pos) @ base_rot
    origin_robot = _transform_mujoco_positions(origin_base.reshape(1, 3))[0]
    bounds_min = vertices_robot.min(axis=0)
    bounds_max = vertices_robot.max(axis=0)
    size = bounds_max - bounds_min
    if np.any(size <= 0):
        raise ValueError(f"Invalid target bounds: {size}")
    return TargetGeometry(origin_robot, (bounds_min + bounds_max) / 2 - origin_robot, size)


def _transform_mujoco_positions(positions):
    from workspace.frame_transform import FrameTransform
    positions = np.asarray(positions, dtype=np.float64)
    if positions.ndim == 1:
        return FrameTransform.mujoco_base_to_robot_base(positions)
    return FrameTransform.transform_positions(positions)


def _get_geom_bounding_vertices(model, geom_id, mujoco):
    geom_type = int(model.geom_type[geom_id])
    size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[geom_id])
        start = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        return np.asarray(model.mesh_vert[start:start + count], dtype=np.float64).copy()
    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        half = size[:3]
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        half = np.repeat(size[0], 3)
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        half = size[:3]
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        half = np.array([size[0], size[0], size[1]])
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        half = np.array([size[0], size[0], size[0] + size[1]])
    else:
        return np.empty((0, 3))
    x, y, z = half
    return np.array([[a, b, c] for a in (-x, x) for b in (-y, y) for c in (-z, z)])


class WorkspaceFilter:
    SUPPORTED_SHAPES = {"box", "sphere", "cylinder"}

    def __init__(self, positions, edges, safety_margin=0.01):
        # Menyimpan graf GNG asli tanpa menghapus node atau edge
        self.positions = np.asarray(positions, dtype=np.float64)
        self.edges = np.asarray(edges, dtype=np.int64)
        self.safety_margin = float(safety_margin)
        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError("positions must have shape (N, 3)")
        if self.edges.size == 0:
            self.edges = self.edges.reshape((0, 2))
        if self.edges.ndim != 2 or self.edges.shape[1] != 2:
            raise ValueError("edges must have shape (M, 2)")
        if not np.all(np.isfinite(self.positions)):
            raise ValueError("positions must be finite")
        if not np.isfinite(self.safety_margin) or self.safety_margin < 0:
            raise ValueError("safety_margin must be non-negative and finite")
        if np.any(self.edges < 0) or np.any(self.edges >= len(self.positions)):
            raise ValueError("edge references an invalid node index")
        self.target_box_center = None
        self.target_box_size = None
        self.obstacles = {}  # Key: (namespace, ID)
        self.valid_nodes = np.ones(len(self.positions), dtype=bool)
        self.valid_edges = np.ones(len(self.edges), dtype=bool)

    @property
    def obstacle(self):
        # Kompatibilitas dengan kode lama yang menggunakan satu obstacle
        return self.obstacles.get(("obstacles", 1))

    def set_target_box(self, center, size):
        # Menerima geometri target dari planning node
        center = self._vector(center, "target center")
        size = self._positive_size(size, "target size")
        changed = (self.target_box_center is None or
                   not np.allclose(center, self.target_box_center, rtol=0, atol=1e-6) or
                   not np.allclose(size, self.target_box_size, rtol=0, atol=1e-6))
        self.target_box_center = center
        self.target_box_size = size
        return changed

    def set_obstacle(self, shape_type, center, size, quaternion=(0, 0, 0, 1),
                     obstacle_id=1, namespace="obstacles"):
        # Menerima atau memperbarui obstacle berdasarkan namespace dan ID
        shape = str(shape_type).lower()
        if shape not in self.SUPPORTED_SHAPES:
            raise ValueError(f"Unsupported obstacle shape: {shape}")
        key = (str(namespace), int(obstacle_id))
        candidate = {
            "shape": shape,
            "center": self._vector(center, "obstacle center"),
            "size": self._positive_size(size, "obstacle size"),
            "rotation": self._quaternion_to_rotation(quaternion),
        }
        previous = self.obstacles.get(key)
        changed = (previous is None or previous["shape"] != shape or
                   any(not np.allclose(previous[name], candidate[name], rtol=0, atol=1e-6)
                       for name in ("center", "size", "rotation")))
        self.obstacles[key] = candidate
        return changed

    def clear_obstacle(self, obstacle_id=1, namespace="obstacles"):
        # Menghapus obstacle tertentu, tanpa memengaruhi obstacle lain
        key = (str(namespace), int(obstacle_id))
        if key not in self.obstacles:
            return False
        del self.obstacles[key]
        return True

    def clear_all_obstacles(self):
        # Menghapus seluruh obstacle
        changed = bool(self.obstacles)
        self.obstacles.clear()
        return changed

    def update(self):
        # Menghitung ulang validity mask saat target atau obstacle berubah
        self.valid_nodes = np.array(
            [not self.is_point_blocked(point) for point in self.positions], dtype=bool
        )
        self.valid_edges = np.zeros(len(self.edges), dtype=bool)
        for i, (a, b) in enumerate(self.edges):
            if self.valid_nodes[a] and self.valid_nodes[b]:
                self.valid_edges[i] = not self.is_segment_blocked(
                    self.positions[a], self.positions[b]
                )
        return self.valid_nodes.copy(), self.valid_edges.copy()

    def is_point_blocked(self, point, include_target=True, include_obstacle=True):
        point = self._vector(point, "point")
        if include_target and self.target_box_center is not None:
            half = self.target_box_size / 2 + self.safety_margin
            if np.all(np.abs(point - self.target_box_center) <= half):
                return True
        if include_obstacle:
            for obstacle in self.obstacles.values():
                local = obstacle["rotation"].T @ (point - obstacle["center"])
                half = obstacle["size"] / 2 + self.safety_margin
                if obstacle["shape"] == "box":
                    if np.all(np.abs(local) <= half):
                        return True
                elif obstacle["shape"] == "sphere":
                    if np.sum((local / half) ** 2) <= 1:
                        return True
                elif obstacle["shape"] == "cylinder":
                    if (np.sum((local[:2] / half[:2]) ** 2) <= 1
                            and abs(local[2]) <= half[2]):
                        return True
        return False

    def is_segment_blocked(self, start, end, include_target=True, include_obstacle=True):
        start = self._vector(start, "segment start")
        end = self._vector(end, "segment end")
        if include_target and self.target_box_center is not None:
            half = self.target_box_size / 2 + self.safety_margin
            if self._segment_intersects_box(start - self.target_box_center,
                                            end - self.target_box_center, half):
                return True
        if include_obstacle:
            for obstacle in self.obstacles.values():
                rot = obstacle["rotation"]
                center = obstacle["center"]
                a = rot.T @ (start - center)
                b = rot.T @ (end - center)
                half = obstacle["size"] / 2 + self.safety_margin
                shape = obstacle["shape"]
                if shape == "box" and self._segment_intersects_box(a, b, half):
                    return True
                if shape == "sphere" and self._segment_intersects_unit_ellipsoid(a / half, b / half):
                    return True
                if shape == "cylinder" and self._segment_intersects_cylinder(a, b, half[:2], half[2]):
                    return True
        return False

    @staticmethod
    def _segment_intersects_box(start, end, half):
        direction = end - start
        t_min, t_max = 0.0, 1.0
        for i in range(3):
            if abs(direction[i]) < 1e-12:
                if abs(start[i]) > half[i]:
                    return False
                continue
            t1 = (-half[i] - start[i]) / direction[i]
            t2 = (half[i] - start[i]) / direction[i]
            t_min = max(t_min, min(t1, t2))
            t_max = min(t_max, max(t1, t2))
            if t_min > t_max:
                return False
        return True

    @staticmethod
    def _segment_intersects_unit_ellipsoid(start, end):
        direction = end - start
        denom = float(np.dot(direction, direction))
        t = 0.0 if denom < 1e-12 else float(np.clip(-np.dot(start, direction) / denom, 0, 1))
        closest = start + t * direction
        return bool(np.dot(closest, closest) <= 1)

    @classmethod
    def _segment_intersects_cylinder(cls, start, end, radii, half_height):
        radial = cls._quadratic_inside_interval(start[:2] / radii, (end[:2] - start[:2]) / radii)
        vertical = cls._linear_inside_interval(start[2], end[2] - start[2], -half_height, half_height)
        return (radial is not None and vertical is not None and
                max(radial[0], vertical[0]) <= min(radial[1], vertical[1]))

    @staticmethod
    def _quadratic_inside_interval(start, direction):
        a = float(np.dot(direction, direction))
        b = 2 * float(np.dot(start, direction))
        c = float(np.dot(start, start)) - 1
        if a < 1e-12:
            return (0.0, 1.0) if c <= 0 else None
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return (0.0, 1.0) if c <= 0 else None
        root = np.sqrt(max(discriminant, 0))
        lower = max(0.0, (-b - root) / (2 * a))
        upper = min(1.0, (-b + root) / (2 * a))
        return (lower, upper) if lower <= upper else None

    @staticmethod
    def _linear_inside_interval(start, direction, lower, upper):
        if abs(direction) < 1e-12:
            return (0.0, 1.0) if lower <= start <= upper else None
        t1 = (lower - start) / direction
        t2 = (upper - start) / direction
        a, b = max(0.0, min(t1, t2)), min(1.0, max(t1, t2))
        return (a, b) if a <= b else None

    @staticmethod
    def _vector(value, name):
        value = np.asarray(value, dtype=np.float64)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain three finite numbers")
        return value.copy()

    @classmethod
    def _positive_size(cls, value, name):
        value = cls._vector(value, name)
        if np.any(value <= 0):
            raise ValueError(f"{name} must be positive")
        return value

    @staticmethod
    def _quaternion_to_rotation(quaternion):
        q = np.asarray(quaternion, dtype=np.float64)
        if q.shape != (4,) or not np.all(np.isfinite(q)):
            raise ValueError("quaternion must have four finite numbers")
        norm = np.linalg.norm(q)
        if norm < 1e-12:
            raise ValueError("quaternion cannot be zero")
        x, y, z, w = q / norm
        return np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ], dtype=np.float64)
