"""Read-only geometric comparisons; nearest positions are not execution progress."""

import numpy as np


def execution_progress_report(path, current_q, next_waypoint, tolerance):
    report = {
        "ordered_next_waypoint": next_waypoint,
        "waypoint_tolerance_rad": tolerance,
        "nearest_is_progress_evidence": False,
        "commanded_path_points": 0 if path is None else len(path),
    }
    if path is None or current_q is None:
        return report
    report["current_q"] = current_q.tolist()
    if 0 <= next_waypoint < len(path):
        report["ordered_next_q"] = path[next_waypoint].tolist()
        report["ordered_next_max_error_rad"] = float(
            np.max(np.abs(path[next_waypoint] - current_q)))
    distances = np.linalg.norm(path - current_q, axis=1)
    nearest = int(np.argmin(distances))
    report["nearest_waypoint_index"] = nearest
    report["nearest_waypoint_l2_error_rad"] = float(distances[nearest])
    if len(path) > 1:
        delta = path[1:] - path[:-1]
        lengths2 = np.sum(delta * delta, axis=1)
        fractions = np.divide(
            np.sum((current_q - path[:-1]) * delta, axis=1), lengths2,
            out=np.zeros_like(lengths2), where=lengths2 > 1e-18)
        fractions = np.clip(fractions, 0., 1.)
        projections = path[:-1] + fractions[:, None] * delta
        errors = np.linalg.norm(projections - current_q, axis=1)
        edge = int(np.argmin(errors))
        report["nearest_segment_index"] = edge
        report["nearest_segment_fraction"] = float(fractions[edge])
        report["nearest_segment_l2_error_rad"] = float(errors[edge])
    return report
