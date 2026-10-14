#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from trajectory_generator import TrajectoryGenerator


def create_test_path():
    return np.array(
        [
            [0.00, 0.00, 0.00],
            [0.10, 0.00, 0.00],
            [0.20, 0.05, 0.00],
            [0.30, 0.10, 0.05],
            [0.40, 0.10, 0.10],
        ],
        dtype=np.float64,
    )


def main():
    path = create_test_path()

    curves = [
        "linear",
        "ease_in",
        "ease_out",
        "smooth_step",
        "smoother_step",
    ]

    speed = 0.1
    distance_step = 0.01

    results = {}

    print("\nTrajectory test")
    print("================")

    for curve_type in curves:
        generator = TrajectoryGenerator(
            speed=speed,
            distance_step=distance_step,
            curve_type=curve_type,
        )

        trajectory = generator.generate(
            path
        )

        results[curve_type] = trajectory

        print(
            f"\nCurve: {curve_type}"
        )

        print(
            f"  Distance : "
            f"{trajectory['total_distance']:.3f} m"
        )

        print(
            f"  Time     : "
            f"{trajectory['total_time']:.3f} s"
        )

        print(
            f"  Samples  : "
            f"{len(trajectory['time'])}"
        )

        print(
            f"  Start    : "
            f"{trajectory['position'][0]}"
        )

        print(
            f"  Goal     : "
            f"{trajectory['position'][-1]}"
        )

    plot_position(
        results
    )

    plot_velocity(
        results
    )

    plt.show()


def plot_position(results):
    plt.figure()

    for curve_type, trajectory in results.items():
        time = trajectory["time"]
        position = trajectory["position"]

        distance = np.linalg.norm(
            position - position[0],
            axis=1,
        )

        plt.plot(
            time,
            distance,
            label=curve_type,
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Distance traveled (m)")
    plt.title("Trajectory Progress")
    plt.grid(True)
    plt.legend()


def plot_velocity(results):
    plt.figure()

    for curve_type, trajectory in results.items():
        time = trajectory["time"]
        velocity = trajectory["velocity"]

        speed = np.linalg.norm(
            velocity,
            axis=1,
        )

        plt.plot(
            time,
            speed,
            label=curve_type,
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Trajectory Velocity")
    plt.grid(True)
    plt.legend()


if __name__ == "__main__":
    main()