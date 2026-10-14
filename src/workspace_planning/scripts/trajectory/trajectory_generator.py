#!/usr/bin/env python3

import numpy as np

from curve import Curve


class TrajectoryGenerator:

    def __init__(
        self,
        speed=0.1,
        distance_step=0.01,
        curve_type="smoother_step",
    ):
        self.speed = float(speed)
        self.distance_step = float(distance_step)
        self.curve = Curve(curve_type)

        if not np.isfinite(self.speed) or self.speed <= 0.0:
            raise ValueError("Speed must be a finite positive value.")

        if (
            not np.isfinite(self.distance_step)
            or self.distance_step <= 0.0
        ):
            raise ValueError(
                "Distance step must be a finite positive value."
            )

    def generate(self, path):
        path = np.asarray(path, dtype=np.float64)

        if path.ndim != 2 or path.shape[1] != 3:
            raise ValueError("Path must have shape (N, 3).")

        if len(path) < 2:
            raise ValueError("Path must contain at least two points.")

        if not np.all(np.isfinite(path)):
            raise ValueError("Path contains non-finite values.")

        path = self._remove_duplicates(path)

        if len(path) < 2:
            raise ValueError(
                "Path must contain at least two distinct points."
            )

        cumulative_distance = self._calculate_cumulative_distance(path)
        total_distance = float(cumulative_distance[-1])

        if total_distance <= 1e-9:
            raise ValueError("Path length is too short.")

        positions = self._sample_path(
            path,
            cumulative_distance,
            total_distance,
        )

        total_time = total_distance / self.speed

        times = self._calculate_times(
            positions,
            total_distance,
            total_time,
        )

        velocities = self._calculate_velocities(
            positions,
            times,
        )

        return {
            "positions": positions,
            "times": times,
            "velocities": velocities,
            "total_distance": total_distance,
            "total_time": total_time,
        }

    @staticmethod
    def _remove_duplicates(path, tolerance=1e-9):
        if len(path) < 2:
            return path

        differences = np.diff(path, axis=0)
        distances = np.linalg.norm(differences, axis=1)

        keep = np.concatenate(
            (
                [True],
                distances > tolerance,
            )
        )

        return path[keep]

    @staticmethod
    def _calculate_cumulative_distance(path):
        segment_vectors = np.diff(path, axis=0)
        segment_lengths = np.linalg.norm(
            segment_vectors,
            axis=1,
        )

        return np.concatenate(
            (
                [0.0],
                np.cumsum(segment_lengths),
            )
        )

    def _sample_path(
        self,
        path,
        cumulative_distance,
        total_distance,
    ):
        distances = np.arange(
            0.0,
            total_distance,
            self.distance_step,
            dtype=np.float64,
        )

        if len(distances) == 0 or not np.isclose(
            distances[0],
            0.0,
        ):
            distances = np.insert(distances, 0, 0.0)

        if not np.isclose(
            distances[-1],
            total_distance,
        ):
            distances = np.append(
                distances,
                total_distance,
            )
        else:
            distances[-1] = total_distance

        positions = np.column_stack(
            [
                np.interp(
                    distances,
                    cumulative_distance,
                    path[:, axis],
                )
                for axis in range(3)
            ]
        )

        positions[0] = path[0]
        positions[-1] = path[-1]

        return positions

    def _calculate_times(
        self,
        positions,
        total_distance,
        total_time,
    ):
        segment_lengths = np.linalg.norm(
            np.diff(positions, axis=0),
            axis=1,
        )

        sampled_distance = np.concatenate(
            (
                [0.0],
                np.cumsum(segment_lengths),
            )
        )

        progress = np.clip(
            sampled_distance / total_distance,
            0.0,
            1.0,
        )

        normalized_time = np.array(
            [
                self._inverse_curve(value)
                for value in progress
            ],
            dtype=np.float64,
        )

        times = normalized_time * total_time

        times[0] = 0.0
        times[-1] = total_time

        return times

    def _inverse_curve(self, target, tolerance=1e-10):
        if target <= 0.0:
            return 0.0

        if target >= 1.0:
            return 1.0

        lower = 0.0
        upper = 1.0

        for _ in range(100):
            middle = (lower + upper) / 2.0
            value = float(self.curve.evaluate(middle))

            if abs(value - target) < tolerance:
                return middle

            if value < target:
                lower = middle
            else:
                upper = middle

        return (lower + upper) / 2.0

    @staticmethod
    def _calculate_velocities(positions, times):
        if len(positions) < 2:
            return np.zeros_like(positions)

        velocities = np.zeros_like(positions)

        time_differences = np.diff(times)

        if np.any(time_differences <= 0.0):
            raise ValueError(
                "Trajectory timestamps must be strictly increasing."
            )

        velocities[0] = (
            positions[1] - positions[0]
        ) / time_differences[0]

        velocities[-1] = (
            positions[-1] - positions[-2]
        ) / time_differences[-1]

        if len(positions) > 2:
            velocities[1:-1] = (
                positions[2:] - positions[:-2]
            ) / (
                times[2:] - times[:-2]
            )[:, np.newaxis]

        return velocities