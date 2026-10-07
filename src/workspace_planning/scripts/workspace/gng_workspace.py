#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent.parent
SCRIPTS_DIR = PACKAGE_DIR / "scripts"

sys.path.insert(0, str(SCRIPTS_DIR))

from config.robot_config import RobotConfig


class GrowingNeuralGas:
    """Growing Neural Gas for workspace representation."""

    def __init__(
        self,
        max_nodes,
        epochs,
        epsilon_b,
        epsilon_n,
        max_edge_age,
        lambda_,
        error_decay,
        random_seed=42,
    ):
        self.max_nodes = max_nodes
        self.epochs = epochs
        self.epsilon_b = epsilon_b
        self.epsilon_n = epsilon_n
        self.max_edge_age = max_edge_age
        self.lambda_ = lambda_
        self.error_decay = error_decay

        self.rng = np.random.default_rng(
            random_seed
        )

        self.nodes = None
        self.errors = None
        self.edges = {}

    def _initialize(self, samples):
        """Initialize two nodes from workspace samples."""

        indices = self.rng.choice(
            len(samples),
            size=2,
            replace=False,
        )

        self.nodes = samples[indices].copy()

        self.errors = np.zeros(
            2,
            dtype=np.float64,
        )

        self.edges = {
            (0, 1): 0,
        }

    def _nearest_nodes(self, sample):
        """Return the nearest and second-nearest nodes."""

        distances = np.linalg.norm(
            self.nodes - sample,
            axis=1,
        )

        nearest = np.argsort(distances)

        return (
            int(nearest[0]),
            int(nearest[1]),
        )

    def _remove_old_edges(self):
        """Remove edges that exceed the maximum age."""

        remove_edges = [
            edge
            for edge, age in self.edges.items()
            if age > self.max_edge_age
        ]

        for edge in remove_edges:
            del self.edges[edge]

    def _remove_isolated_nodes(self):
        """Remove nodes that no longer have edges."""

        connected = set()

        for first, second in self.edges:
            connected.add(first)
            connected.add(second)

        if len(connected) == len(self.nodes):
            return

        keep = sorted(connected)

        if len(keep) < 2:
            return

        mapping = {
            old: new
            for new, old in enumerate(keep)
        }

        self.nodes = self.nodes[keep]
        self.errors = self.errors[keep]

        new_edges = {}

        for first, second in self.edges:
            if (
                first in mapping
                and second in mapping
            ):
                new_edge = (
                    mapping[first],
                    mapping[second],
                )

                new_edges[
                    tuple(sorted(new_edge))
                ] = self.edges[
                    (first, second)
                ]

        self.edges = new_edges

    def _increment_neighbor_ages(self, nearest):
        """Increase the age of edges connected to the winner."""

        for edge in list(self.edges):
            first, second = edge

            if first == nearest or second == nearest:
                self.edges[edge] += 1

    def _connect_nodes(self, first, second):
        """Create or reset an edge between two nodes."""

        edge = tuple(
            sorted(
                (
                    first,
                    second,
                )
            )
        )

        self.edges[edge] = 0

    def _insert_node(self):
        """Insert a new node between high-error nodes."""

        if len(self.nodes) >= self.max_nodes:
            return

        if len(self.edges) == 0:
            return

        first = int(
            np.argmax(self.errors)
        )

        neighbor_candidates = []

        for edge in self.edges:
            a, b = edge

            if a == first:
                neighbor_candidates.append(b)
            elif b == first:
                neighbor_candidates.append(a)

        if not neighbor_candidates:
            return

        second = max(
            neighbor_candidates,
            key=lambda index: self.errors[index],
        )

        new_position = (
            self.nodes[first]
            + self.nodes[second]
        ) / 2.0

        new_index = len(self.nodes)

        self.nodes = np.vstack(
            [
                self.nodes,
                new_position,
            ]
        )

        new_error = (
            self.errors[first]
            + self.errors[second]
        ) / 2.0

        self.errors = np.append(
            self.errors,
            new_error,
        )

        old_edge = tuple(
            sorted(
                (
                    first,
                    second,
                )
            )
        )

        if old_edge in self.edges:
            del self.edges[old_edge]

        self._connect_nodes(
            first,
            new_index,
        )

        self._connect_nodes(
            second,
            new_index,
        )

        self.errors[first] *= 0.5
        self.errors[second] *= 0.5

    def fit(self, samples):
        """Train GNG using workspace samples."""

        samples = np.asarray(
            samples,
            dtype=np.float64,
        )

        if samples.ndim != 2:
            raise ValueError(
                "Samples must have shape (N, D)."
            )

        if len(samples) < 2:
            raise ValueError(
                "At least two samples are required."
            )

        self._initialize(samples)

        for epoch in range(self.epochs):
            order = self.rng.permutation(
                len(samples)
            )

            for step, index in enumerate(order):
                sample = samples[index]

                nearest, second = (
                    self._nearest_nodes(
                        sample
                    )
                )

                distance = np.linalg.norm(
                    sample
                    - self.nodes[nearest]
                )

                self.errors[nearest] += (
                    distance ** 2
                )

                self._increment_neighbor_ages(
                    nearest
                )

                self.nodes[nearest] += (
                    self.epsilon_b
                    * (
                        sample
                        - self.nodes[nearest]
                    )
                )

                neighbors = []

                for edge in self.edges:
                    first, second_node = edge

                    if first == nearest:
                        neighbors.append(
                            second_node
                        )
                    elif second_node == nearest:
                        neighbors.append(
                            first
                        )

                for neighbor in neighbors:
                    self.nodes[neighbor] += (
                        self.epsilon_n
                        * (
                            sample
                            - self.nodes[neighbor]
                        )
                    )

                self._connect_nodes(
                    nearest,
                    second,
                )

                self._remove_old_edges()
                self._remove_isolated_nodes()

                if (
                    self.lambda_ > 0
                    and step > 0
                    and step % self.lambda_ == 0
                    and len(self.nodes)
                    < self.max_nodes
                ):
                    self._insert_node()

                self.errors *= (
                    self.error_decay
                )

            print(
                f"Epoch "
                f"{epoch + 1:02d}/"
                f"{self.epochs:02d} | "
                f"Nodes: {len(self.nodes):4d} | "
                f"Edges: {len(self.edges):4d}"
            )

        return self.nodes, self.edges


def nearest_sample_indices(
    prototypes,
    samples,
):
    """Map GNG nodes to the nearest raw workspace samples."""

    indices = []

    for prototype in prototypes:
        distances = np.linalg.norm(
            samples - prototype,
            axis=1,
        )

        indices.append(
            int(np.argmin(distances))
        )

    return np.asarray(
        indices,
        dtype=np.int64,
    )


def main():
    package_dir = PACKAGE_DIR

    config_path = (
        package_dir
        / "config"
        / "robot.yaml"
    )

    data_dir = (
        package_dir
        / "data"
    )

    input_path = (
        data_dir
        / "raw_workspace.npz"
    )

    output_path = (
        data_dir
        / "workspace_graph.npz"
    )

    config = RobotConfig(
        config_path
    )

    if not input_path.exists():
        raise FileNotFoundError(
            f"Workspace data not found:\n"
            f"{input_path}\n\n"
            "Run generate_workspace.py first."
        )

    data = np.load(
        input_path,
        allow_pickle=True,
    )

    eof = data["eof"]

    print("Workspace GNG")
    print()
    print(
        f"Input samples : "
        f"{len(eof)}"
    )
    print(
        f"Max nodes     : "
        f"{config.gng_max_nodes}"
    )
    print(
        f"Epochs        : "
        f"{config.gng_epochs}"
    )
    print()

    gng = GrowingNeuralGas(
        max_nodes=config.gng_max_nodes,
        epochs=config.gng_epochs,
        epsilon_b=config.gng_epsilon_b,
        epsilon_n=config.gng_epsilon_n,
        max_edge_age=config.gng_max_edge_age,
        lambda_=config.gng_lambda,
        error_decay=config.gng_error_decay,
        random_seed=config.workspace_random_seed,
    )

    prototypes, edges = gng.fit(
        eof
    )

    sample_indices = (
        nearest_sample_indices(
            prototypes,
            eof,
        )
    )

    graph_positions = eof[
        sample_indices
    ]

    graph_q = data["q"][
        sample_indices
    ]

    graph_joint_positions = data[
        "joint_positions"
    ][
        sample_indices
    ]

    edge_array = np.asarray(
        list(edges.keys()),
        dtype=np.int64,
    )

    if edge_array.size == 0:
        edge_array = np.empty(
            (0, 2),
            dtype=np.int64,
        )

    np.savez_compressed(
        output_path,
        positions=graph_positions,
        q=graph_q,
        joint_positions=graph_joint_positions,
        prototype_positions=prototypes,
        errors=gng.errors,
        edges=edge_array,
        sample_indices=sample_indices,
        coordinate_frame=data[
            "coordinate_frame"
        ],
        source_coordinate_frame=data[
            "source_coordinate_frame"
        ],
        joint_names=data[
            "joint_names"
        ],
        body_names=data[
            "body_names"
        ],
        joint_limits=data[
            "joint_limits"
        ],
    )

    print()
    print("GNG completed.")
    print(
        f"Nodes         : "
        f"{len(prototypes)}"
    )
    print(
        f"Edges         : "
        f"{len(edges)}"
    )
    print(
        f"Output        : "
        f"{output_path}"
    )


if __name__ == "__main__":
    main()