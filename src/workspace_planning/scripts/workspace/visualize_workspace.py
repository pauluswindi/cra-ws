#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent.parent

DATA_DIR = PACKAGE_DIR / "data"

RAW_PATH = (
    DATA_DIR
    / "raw_workspace.npz"
)

GNG_PATH = (
    DATA_DIR
    / "workspace_graph.npz"
)


RAW_COLOR = "gray"
GNG_NODE_COLOR = "green"
GNG_EDGE_COLOR = "orange"
AXIS_X_COLOR = "red"
AXIS_Y_COLOR = "green"
AXIS_Z_COLOR = "blue"


def set_equal_axes(ax, points):
    """Set equal scale for 3D workspace plots."""

    minimum = np.min(
        points,
        axis=0,
    )

    maximum = np.max(
        points,
        axis=0,
    )

    center = (
        minimum + maximum
    ) / 2.0

    radius = (
        np.max(
            maximum - minimum
        )
        / 2.0
    )

    ax.set_xlim(
        center[0] - radius,
        center[0] + radius,
    )

    ax.set_ylim(
        center[1] - radius,
        center[1] + radius,
    )

    ax.set_zlim(
        center[2] - radius,
        center[2] + radius,
    )


def draw_robot_axes_3d(ax, length):
    """Draw the robot base coordinate axes."""

    origin = np.zeros(3)

    ax.quiver(
        *origin,
        length,
        0,
        0,
        color=AXIS_X_COLOR,
        linewidth=2,
        arrow_length_ratio=0.08,
    )

    ax.quiver(
        *origin,
        0,
        length,
        0,
        color=AXIS_Y_COLOR,
        linewidth=2,
        arrow_length_ratio=0.08,
    )

    ax.quiver(
        *origin,
        0,
        0,
        length,
        color=AXIS_Z_COLOR,
        linewidth=2,
        arrow_length_ratio=0.08,
    )

    ax.text(
        length,
        0,
        0,
        "+X Front",
    )

    ax.text(
        0,
        length,
        0,
        "+Y Left",
    )

    ax.text(
        0,
        0,
        length,
        "+Z Up",
    )


def draw_2d_axes(ax):
    """Draw the robot base axes on a 2D projection."""

    ax.axhline(
        0,
        linewidth=1,
    )

    ax.axvline(
        0,
        linewidth=1,
    )


def plot_raw_3d(eof):
    """Plot the raw workspace in 3D."""

    figure = plt.figure(
        figsize=(9, 7)
    )

    ax = figure.add_subplot(
        111,
        projection="3d",
    )

    ax.scatter(
        eof[:, 0],
        eof[:, 1],
        eof[:, 2],
        s=2,
        alpha=0.25,
        color=RAW_COLOR,
    )

    draw_robot_axes_3d(
        ax,
        np.max(
            np.ptp(eof, axis=0)
        ) * 0.2,
    )

    set_equal_axes(
        ax,
        eof,
    )

    ax.set_xlabel(
        "X Front / Back"
    )

    ax.set_ylabel(
        "Y Left / Right"
    )

    ax.set_zlabel(
        "Z Up / Down"
    )

    ax.set_title(
        "Raw Workspace"
    )

    figure.tight_layout()


def plot_projections(eof):
    """Plot all workspace projections in one figure."""

    figure, axes = plt.subplots(
        1,
        3,
        figsize=(18, 5),
    )

    ax_xy = axes[0]

    ax_xy.scatter(
        eof[:, 0],
        eof[:, 1],
        s=2,
        alpha=0.25,
        color=RAW_COLOR,
    )

    draw_2d_axes(
        ax_xy
    )

    ax_xy.set_xlabel(
        "X Front / Back"
    )

    ax_xy.set_ylabel(
        "Y Left / Right"
    )

    ax_xy.set_title(
        "XY Projection"
    )

    ax_xy.set_aspect(
        "equal",
        adjustable="box",
    )

    ax_xz = axes[1]

    ax_xz.scatter(
        eof[:, 0],
        eof[:, 2],
        s=2,
        alpha=0.25,
        color=RAW_COLOR,
    )

    draw_2d_axes(
        ax_xz
    )

    ax_xz.set_xlabel(
        "X Front / Back"
    )

    ax_xz.set_ylabel(
        "Z Up / Down"
    )

    ax_xz.set_title(
        "XZ Projection"
    )

    ax_xz.set_aspect(
        "equal",
        adjustable="box",
    )

    ax_yz = axes[2]

    ax_yz.scatter(
        eof[:, 1],
        eof[:, 2],
        s=2,
        alpha=0.25,
        color=RAW_COLOR,
    )

    draw_2d_axes(
        ax_yz
    )

    ax_yz.set_xlabel(
        "Y Left / Right"
    )

    ax_yz.set_ylabel(
        "Z Up / Down"
    )

    ax_yz.set_title(
        "YZ Projection"
    )

    ax_yz.set_aspect(
        "equal",
        adjustable="box",
    )

    figure.suptitle(
        "Workspace Projections",
        fontsize=14,
    )

    figure.tight_layout()


def plot_raw_and_gng(
    eof,
    graph_positions,
    edges,
):
    """Plot raw workspace together with GNG graph."""

    figure = plt.figure(
        figsize=(9, 7)
    )

    ax = figure.add_subplot(
        111,
        projection="3d",
    )

    ax.scatter(
        eof[:, 0],
        eof[:, 1],
        eof[:, 2],
        s=1,
        alpha=0.08,
        color=RAW_COLOR,
    )

    for first, second in edges:
        points = graph_positions[
            [first, second]
        ]

        ax.plot(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            linewidth=1,
            color=GNG_EDGE_COLOR,
            alpha=0.7,
        )

    ax.scatter(
        graph_positions[:, 0],
        graph_positions[:, 1],
        graph_positions[:, 2],
        s=12,
        color=GNG_NODE_COLOR,
        alpha=0.9,
    )

    draw_robot_axes_3d(
        ax,
        np.max(
            np.ptp(eof, axis=0)
        ) * 0.2,
    )

    set_equal_axes(
        ax,
        eof,
    )

    ax.set_xlabel(
        "X Front / Back"
    )

    ax.set_ylabel(
        "Y Left / Right"
    )

    ax.set_zlabel(
        "Z Up / Down"
    )

    ax.set_title(
        "Raw Workspace + GNG"
    )

    figure.tight_layout()


def plot_gng_only(
    graph_positions,
    edges,
):
    """Plot only the GNG graph."""

    figure = plt.figure(
        figsize=(9, 7)
    )

    ax = figure.add_subplot(
        111,
        projection="3d",
    )

    for first, second in edges:
        points = graph_positions[
            [first, second]
        ]

        ax.plot(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            linewidth=1,
            color=GNG_EDGE_COLOR,
            alpha=0.7,
        )

    ax.scatter(
        graph_positions[:, 0],
        graph_positions[:, 1],
        graph_positions[:, 2],
        s=16,
        color=GNG_NODE_COLOR,
        alpha=0.9,
    )

    draw_robot_axes_3d(
        ax,
        np.max(
            np.ptp(graph_positions, axis=0)
        ) * 0.2,
    )

    set_equal_axes(
        ax,
        graph_positions,
    )

    ax.set_xlabel(
        "X Front / Back"
    )

    ax.set_ylabel(
        "Y Left / Right"
    )

    ax.set_zlabel(
        "Z Up / Down"
    )

    ax.set_title(
        "GNG Workspace Graph"
    )

    figure.tight_layout()


def main():
    if not RAW_PATH.exists():
        raise FileNotFoundError(
            f"Raw workspace not found:\n"
            f"{RAW_PATH}"
        )

    if not GNG_PATH.exists():
        raise FileNotFoundError(
            f"GNG workspace not found:\n"
            f"{GNG_PATH}\n\n"
            "Run gng_workspace.py first."
        )

    raw_data = np.load(
        RAW_PATH,
        allow_pickle=True,
    )

    gng_data = np.load(
        GNG_PATH,
        allow_pickle=True,
    )

    eof = raw_data["eof"]

    graph_positions = gng_data[
        "positions"
    ]

    edges = gng_data[
        "edges"
    ]

    coordinate_frame = str(
        raw_data["coordinate_frame"]
    )

    print("Workspace Visualization")
    print()
    print(
        f"Coordinate frame : "
        f"{coordinate_frame}"
    )
    print(
        f"Raw samples      : "
        f"{len(eof)}"
    )
    print(
        f"GNG nodes        : "
        f"{len(graph_positions)}"
    )
    print(
        f"GNG edges        : "
        f"{len(edges)}"
    )
    print()

    plot_raw_3d(
        eof
    )

    plot_projections(
        eof
    )

    plot_raw_and_gng(
        eof,
        graph_positions,
        edges,
    )

    plot_gng_only(
        graph_positions,
        edges,
    )

    plt.show()


if __name__ == "__main__":
    main()