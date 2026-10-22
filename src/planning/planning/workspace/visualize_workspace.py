import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_workspace(path):
    if not path.is_file():
        raise FileNotFoundError(f"Workspace file not found: {path}")

    with np.load(path, allow_pickle=False) as data:
        if "eof" not in data.files:
            raise ValueError("Workspace data does not contain 'eof'")

        positions = data["eof"]

    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("Workspace EOF must have shape (N, 3)")

    if not np.isfinite(positions).all():
        raise ValueError("Workspace contains invalid coordinates")

    return positions


def load_gng(path):
    if not path.is_file():
        raise FileNotFoundError(f"GNG graph not found: {path}")

    with np.load(path, allow_pickle=False) as data:
        position_key = (
            "prototype_positions"
            if "prototype_positions" in data.files
            else "positions"
        )

        if position_key not in data.files or "edges" not in data.files:
            raise ValueError("GNG graph requires positions and edges")

        positions = data[position_key]
        edges = data["edges"]

    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("GNG nodes must have shape (N, 3)")

    if edges.ndim != 2 or edges.shape[1] != 2:
        raise ValueError("GNG edges must have shape (E, 2)")

    if not np.isfinite(positions).all():
        raise ValueError("GNG nodes contain invalid coordinates")

    if len(edges):
        if edges.min() < 0 or edges.max() >= len(positions):
            raise ValueError("GNG edges contain invalid node indices")

    return positions, edges


def sample_workspace(positions, max_points, seed):
    if len(positions) <= max_points:
        return positions

    rng = np.random.default_rng(seed)

    indices = rng.choice(
        len(positions),
        size=max_points,
        replace=False,
    )

    return positions[indices]


def set_equal_axes(ax, positions):
    minimum = positions.min(axis=0)
    maximum = positions.max(axis=0)

    center = (minimum + maximum) / 2.0
    radius = max(float(np.max(maximum - minimum)) / 2.0, 0.01)

    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)

    ax.set_box_aspect((1, 1, 1))


def configure_axes(ax, positions, title):
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    set_equal_axes(ax, positions)

    ax.view_init(elev=25, azim=-60)


def plot_workspace(ax, positions):
    ax.scatter(
        positions[:, 0],
        positions[:, 1],
        positions[:, 2],
        s=1,
        alpha=0.15,
        color="#5DADE2",
        rasterized=True,
        label="Workspace EOF",
    )


def plot_gng(ax, positions, edges):
    if len(edges):
        segments = positions[edges]

        from mpl_toolkits.mplot3d.art3d import Line3DCollection

        collection = Line3DCollection(
            segments,
            colors="#F39C12",
            linewidths=0.8,
            alpha=0.7,
        )       

        ax.add_collection3d(collection)

        ax.scatter(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            s=10,
            color="#2ECC71",
            depthshade=False,
            label="GNG Nodes",
        )


def visualize(workspace_path, graph_path, mode, max_points, seed, save_path):
    workspace = None
    graph_positions = None
    graph_edges = None

    if mode in ("workspace", "both"):
        workspace = load_workspace(workspace_path)

    if mode in ("gng", "both"):
        graph_positions, graph_edges = load_gng(graph_path)

    if workspace is not None:
        print(f"Workspace samples : {len(workspace)}", flush=True)

        workspace = sample_workspace(
            workspace,
            max_points,
            seed,
        )

        print(f"Displayed samples : {len(workspace)}", flush=True)

    if graph_positions is not None:
        print(f"GNG nodes         : {len(graph_positions)}", flush=True)
        print(f"GNG edges         : {len(graph_edges)}", flush=True)

    figure = plt.figure(figsize=(12, 8))
    ax = figure.add_subplot(111, projection="3d")

    all_positions = []

    if workspace is not None:
        plot_workspace(ax, workspace)
        all_positions.append(workspace)

    if graph_positions is not None:
        plot_gng(ax, graph_positions, graph_edges)
        all_positions.append(graph_positions)

    positions = np.vstack(all_positions)

    titles = {
        "workspace": "CRA - EOF Workspace",
        "gng": "CRA - GNG Workspace Topology",
        "both": "CRA - EOF Workspace and GNG Topology",
    }

    configure_axes(ax, positions, titles[mode])

    ax.legend(loc="upper right")

    figure.tight_layout()

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)

        figure.savefig(
            save_path,
            dpi=200,
            bbox_inches="tight",
        )

        print(f"Image saved       : {save_path}", flush=True)

    if save_path is None:
        plt.show()
    else:
        plt.close(figure)


def main():
    default_data_dir = (
        Path.home()
        / "living-lab"
        / "cra_ws"
        / "src"
        / "planning"
        / "data"
    )

    parser = argparse.ArgumentParser(
        description="Visualize CRA workspace and GNG graph"
    )

    parser.add_argument(
        "--workspace",
        type=Path,
        default=default_data_dir / "raw_workspace.npz",
    )

    parser.add_argument(
        "--graph",
        type=Path,
        default=default_data_dir / "workspace_graph.npz",
    )

    parser.add_argument(
        "--mode",
        choices=("workspace", "gng", "both"),
        default="both",
    )

    parser.add_argument(
        "--max-points",
        type=int,
        default=10000,
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
    )

    parser.add_argument(
        "--save",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    if args.max_points < 1:
        parser.error("--max-points must be positive")

    visualize(
        workspace_path=args.workspace.expanduser().resolve(),
        graph_path=args.graph.expanduser().resolve(),
        mode=args.mode,
        max_points=args.max_points,
        seed=args.seed,
        save_path=(
            args.save.expanduser().resolve()
            if args.save is not None
            else None
        ),
    )


if __name__ == "__main__":
    main()