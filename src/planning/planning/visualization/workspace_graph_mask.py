"""Display-only XYZ occlusion of a stored graph, never robot-motion validity."""

import time

import numpy as np

from planning.algorithms.human_collision import segment_box_distances


def _point_segment_distance_sq(points, starts, ends):
    axis = ends - starts
    squared = np.sum(axis * axis, axis=-1)
    alpha = np.divide(np.sum((points - starts) * axis, axis=-1), squared,
                      out=np.zeros(np.broadcast_shapes(points.shape[:-1], squared.shape)),
                      where=squared > 0.)
    delta = points - starts - np.clip(alpha, 0., 1.)[..., None] * axis
    return np.sum(delta * delta, axis=-1)


def segment_distances(starts, ends, other_start, other_end):
    """Exact finite-segment distances, including parallel/point segments.

    The constrained minimum lies on one of four endpoint boundaries or at
    the unconstrained interior solution. Cross products avoid cancellation
    in the near-parallel denominator.
    """
    a, b, c, d = (np.asarray(v, dtype=float) for v in
                  (starts, ends, other_start, other_end))
    candidates = [
        _point_segment_distance_sq(a, c, d),
        _point_segment_distance_sq(b, c, d),
        _point_segment_distance_sq(c, a, b),
        _point_segment_distance_sq(d, a, b),
    ]
    u, v, offset = b - a, d - c, c - a
    cross = np.cross(u, v)
    denominator = np.sum(cross * cross, axis=-1)
    s = np.divide(np.sum(np.cross(offset, v) * cross, axis=-1), denominator,
                  out=np.zeros_like(denominator), where=denominator > 0.)
    t = np.divide(np.sum(np.cross(offset, u) * cross, axis=-1), denominator,
                  out=np.zeros_like(denominator), where=denominator > 0.)
    interior = ((denominator > 0.) & (s >= 0.) & (s <= 1.) & (t >= 0.) & (t <= 1.))
    separation = a + s[..., None] * u - c - t[..., None] * v
    candidates.append(np.where(interior, np.sum(separation * separation, axis=-1), np.inf))
    return np.sqrt(np.minimum.reduce(candidates))


def workspace_graph_period(planning):
    settings = planning.get('visualization', {})
    if not isinstance(settings, dict):
        raise ValueError('planning.visualization must be a mapping')
    rate = settings.get('workspace_graph_rate_hz', 2.)
    if (isinstance(rate, bool) or not isinstance(rate, (int, float))
            or not np.isfinite(rate) or rate <= 0.):
        raise ValueError('planning.visualization.workspace_graph_rate_hz must be positive')
    return 1. / float(rate)


class WorkspaceGraphMask:
    """Cache immutable graph geometry and one obstacle signature.

    Capsules use CRA base_link meters: (start, end, radius). Boxes use
    (center, local-to-base rotation, half extents). No robot configuration,
    collision checker, MuJoCo state, executor or worker is owned here.
    """

    KNOWN = ('OBSERVED', 'EMPTY', 'HUMAN_DISABLED')

    def __init__(self, positions, edges):
        self.positions = np.array(positions, dtype=float, copy=True)
        self.edges = np.array(edges, dtype=int, copy=True).reshape(-1, 2)
        if (self.positions.ndim != 2 or self.positions.shape[1] != 3
                or not np.isfinite(self.positions).all()):
            raise ValueError('Invalid workspace graph positions')
        if self.edges.size and (self.edges.min() < 0 or self.edges.max() >= len(self.positions)):
            raise ValueError('Invalid workspace graph edge indices')
        self.positions.flags.writeable = self.edges.flags.writeable = False
        # Nodes are zero-length segments, so nodes and edges share one exact query.
        self.starts = np.vstack((self.positions, self.positions[self.edges[:, 0]]))
        self.ends = np.vstack((self.positions, self.positions[self.edges[:, 1]]))
        self.minimum = np.minimum(self.starts, self.ends)
        self.maximum = np.maximum(self.starts, self.ends)
        self.signature = self.latest = None
        self.calculations = 0

    def update(self, capsules=(), boxes=(), status='OBSERVED'):
        """Return changed visibility only; identical scenes perform no geometry work."""
        signature = (status, tuple((tuple(a), tuple(b), float(r)) for a, b, r in capsules),
                     tuple((tuple(c), tuple(np.asarray(rot).ravel()), tuple(h))
                           for c, rot, h in boxes))
        if signature == self.signature:
            return None
        began = time.perf_counter()
        visible = np.ones(len(self.starts), dtype=bool)
        known = status in self.KNOWN
        # Callers supply only available geometry. Missing human data must not
        # disable masking of independently fresh target/simulation boxes.
        self.calculations += 1
        for a, b, radius in capsules:
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            if (a.shape != (3,) or b.shape != (3,) or not np.isfinite([a, b]).all()
                    or not np.isfinite(radius) or radius <= 0.):
                raise ValueError('Invalid display capsule')
            # Broad phase rejects graph segments outside the capsule AABB.
            near = visible & np.all(
                (self.maximum >= np.minimum(a, b) - radius)
                & (self.minimum <= np.maximum(a, b) + radius), axis=1)
            ids = np.flatnonzero(near)
            if len(ids):
                visible[ids] = segment_distances(
                    self.starts[ids], self.ends[ids], a, b) > radius
        for center, rotation, half in boxes:
            center, rotation, half = (np.asarray(v, dtype=float)
                                      for v in (center, rotation, half))
            if (center.shape != (3,) or rotation.shape != (3, 3) or half.shape != (3,)
                    or not all(np.isfinite(v).all() for v in (center, rotation, half))
                    or np.any(half <= 0.) or not np.allclose(
                        rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0.)):
                raise ValueError('Invalid display box')
            extent = np.abs(rotation) @ half
            near = visible & np.all((self.maximum >= center - extent)
                                    & (self.minimum <= center + extent), axis=1)
            ids = np.flatnonzero(near)
            if len(ids):
                local_a = (self.starts[ids] - center) @ rotation
                local_b = (self.ends[ids] - center) @ rotation
                visible[ids] = segment_box_distances(local_a, local_b, half) > 1e-12
        self.signature = signature
        nodes = visible[:len(self.positions)]
        edges = visible[len(self.positions):]
        edges &= nodes[self.edges[:, 0]] & nodes[self.edges[:, 1]]
        result = dict(visible_node_mask=nodes, visible_edge_mask=edges, status=status,
                      elapsed_s=time.perf_counter() - began, known=known)
        if (self.latest is not None and status == self.latest['status']
                and np.array_equal(nodes, self.latest['visible_node_mask'])
                and np.array_equal(edges, self.latest['visible_edge_mask'])):
            return None
        self.latest = result
        return result
