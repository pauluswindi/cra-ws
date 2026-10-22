import argparse
from pathlib import Path

import numpy as np
import yaml

from planning.gng.joint_graph import build_joint_graph
from ament_index_python.packages import get_package_share_directory


class GrowingNeuralGas:
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

        self.rng = np.random.default_rng(random_seed)

        self.nodes = None
        self.errors = None
        self.edges = {}

    def _initialize(self, samples):
        indices = self.rng.choice(
            len(samples),
            size=2,
            replace=False,
        )

        self.nodes = samples[indices].copy()
        self.errors = np.zeros(2, dtype=np.float64)
        self.edges = {(0, 1): 0}

    def _nearest_nodes(self, sample):
        distances = np.linalg.norm(
            self.nodes - sample,
            axis=1,
        )

        nearest = np.argsort(distances)

        return int(nearest[0]), int(nearest[1])

    def _increment_neighbor_ages(self, nearest):
        for edge in list(self.edges):
            if nearest in edge:
                self.edges[edge] += 1

    def _connect_nodes(self, first, second):
        edge = tuple(sorted((first, second)))
        self.edges[edge] = 0

    def _remove_old_edges(self):
        old_edges = [
            edge
            for edge, age in self.edges.items()
            if age > self.max_edge_age
        ]

        for edge in old_edges:
            del self.edges[edge]

    def _remove_isolated_nodes(self):
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

        for edge, age in self.edges.items():
            first, second = edge

            if first in mapping and second in mapping:
                new_edge = tuple(sorted((
                    mapping[first],
                    mapping[second],
                )))

                new_edges[new_edge] = age

        self.edges = new_edges

    def _insert_node(self):
        if len(self.nodes) >= self.max_nodes:
            return

        if not self.edges:
            return

        first = int(np.argmax(self.errors))

        neighbors = []

        for a, b in self.edges:
            if a == first:
                neighbors.append(b)
            elif b == first:
                neighbors.append(a)

        if not neighbors:
            return

        second = max(
            neighbors,
            key=lambda index: self.errors[index],
        )

        new_position = (
            self.nodes[first] + self.nodes[second]
        ) / 2.0

        new_error = (
            self.errors[first] + self.errors[second]
        ) / 2.0

        new_index = len(self.nodes)

        self.nodes = np.vstack([
            self.nodes,
            new_position,
        ])

        self.errors = np.append(
            self.errors,
            new_error,
        )

        old_edge = tuple(sorted((first, second)))
        self.edges.pop(old_edge, None)

        self._connect_nodes(first, new_index)
        self._connect_nodes(second, new_index)

        self.errors[first] *= 0.5
        self.errors[second] *= 0.5

    def fit(self, samples):
        samples = np.asarray(
            samples,
            dtype=np.float64,
        )

        if samples.ndim != 2 or samples.shape[1] != 3:
            raise ValueError(
                "Workspace samples must have shape (N, 3)"
            )

        if len(samples) < 2:
            raise ValueError(
                "At least two workspace samples are required"
            )

        if not np.all(np.isfinite(samples)):
            raise ValueError(
                "Workspace samples contain invalid values"
            )

        self._initialize(samples)

        for epoch in range(self.epochs):
            order = self.rng.permutation(len(samples))

            for step, index in enumerate(order):
                sample = samples[index]

                nearest, second = self._nearest_nodes(
                    sample
                )

                distance = np.linalg.norm(
                    sample - self.nodes[nearest]
                )

                self.errors[nearest] += distance ** 2

                self._increment_neighbor_ages(nearest)

                self.nodes[nearest] += (
                    self.epsilon_b
                    * (sample - self.nodes[nearest])
                )

                neighbors = []

                for a, b in self.edges:
                    if a == nearest:
                        neighbors.append(b)
                    elif b == nearest:
                        neighbors.append(a)

                for neighbor in neighbors:
                    self.nodes[neighbor] += (
                        self.epsilon_n
                        * (sample - self.nodes[neighbor])
                    )

                self._connect_nodes(nearest, second)

                self._remove_old_edges()
                self._remove_isolated_nodes()

                if (
                    self.lambda_ > 0
                    and step > 0
                    and step % self.lambda_ == 0
                    and len(self.nodes) < self.max_nodes
                ):
                    self._insert_node()

                self.errors *= self.error_decay

            print(
                f"Epoch {epoch + 1}/{self.epochs} | "
                f"Nodes: {len(self.nodes)} | "
                f"Edges: {len(self.edges)}",
                flush=True,
            )

        return self.nodes, self.edges


def nearest_sample_indices(prototypes, samples):
    indices = np.empty(
        len(prototypes),
        dtype=np.int64,
    )

    for index, prototype in enumerate(prototypes):
        distances = np.linalg.norm(
            samples - prototype,
            axis=1,
        )

        indices[index] = int(np.argmin(distances))

    return indices


def load_configuration():
    cra_share = Path(
        get_package_share_directory("cra")
    )

    config_path = cra_share / "config" / "robot.yaml"

    with config_path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_gng(config, input_path, output_path):
    with np.load(input_path, allow_pickle=False) as data:
        required = (
            "q",
            "eof",
            "joint_positions",
            "joint_names",
            "body_names",
            "joint_limits",
            "coordinate_frame",
            "source_coordinate_frame",
        )

        missing = [
            name for name in required
            if name not in data.files
        ]

        if missing:
            raise ValueError(
                f"Missing workspace data: {missing}"
            )

        q = data["q"]
        eof = data["eof"]
        joint_positions = data["joint_positions"]

        metadata = {
            name: data[name].copy()
            for name in (
                "joint_names",
                "body_names",
                "joint_limits",
                "coordinate_frame",
                "source_coordinate_frame",
            )
        }

    if q.ndim != 2 or q.shape[1] != 5:
        raise ValueError("Expected five-joint workspace data")

    if eof.shape != (len(q), 3):
        raise ValueError("Invalid EOF sample shape")

    if (
        joint_positions.ndim != 3
        or joint_positions.shape[0] != len(q)
        or joint_positions.shape[2] != 3
    ):
        raise ValueError("Invalid link position data")

    gng_config = config["gng"]

    gng = GrowingNeuralGas(
        max_nodes=int(gng_config["max_nodes"]),
        epochs=int(gng_config["epochs"]),
        epsilon_b=float(gng_config["epsilon_b"]),
        epsilon_n=float(gng_config["epsilon_n"]),
        max_edge_age=int(gng_config["max_edge_age"]),
        lambda_=int(gng_config["lambda"]),
        error_decay=float(gng_config["error_decay"]),
        random_seed=int(config["workspace"]["random_seed"]),
    )

    print("CRA Workspace GNG", flush=True)
    print(f"Samples   : {len(eof)}", flush=True)
    print(f"Max nodes : {gng.max_nodes}", flush=True)
    print(f"Epochs    : {gng.epochs}", flush=True)

    prototypes, edges = gng.fit(eof)

    sample_indices = nearest_sample_indices(
        prototypes,
        eof,
    )

    edge_array = np.asarray(
        list(edges.keys()),
        dtype=np.int64,
    ).reshape(-1, 2)

    print("\nBuilding joint graph...", flush=True)

    joint_config = config.get("planning", {}).get(
        "joint_graph", {}
    )

    joint_graph = build_joint_graph(
        prototypes=prototypes,
        gng_edges=edge_array,
        workspace_eof=eof,
        workspace_q=q,
        max_candidates=int(
            joint_config.get("max_candidates", 4)
        ),
        nearest_pool=int(
            joint_config.get("nearest_pool", 64)
        ),
        max_joint_delta=float(
            joint_config.get("max_joint_delta", 0.6)
        ),
    )

    # Save GNG data and augmented joint states together.
    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    np.savez_compressed(
        output_path,
        positions=eof[sample_indices],
        q=q[sample_indices],
        joint_positions=joint_positions[sample_indices],
        prototype_positions=prototypes,
        errors=gng.errors,
        edges=edge_array,
        sample_indices=sample_indices,
        **metadata,
        **joint_graph,
    )

    print("\nGNG generation completed", flush=True)
    print(f"GNG nodes    : {len(prototypes)}", flush=True)
    print(f"GNG edges    : {len(edge_array)}", flush=True)
    print(
        f"Joint states : {len(joint_graph['joint_q'])}",
        flush=True,
    )
    print(
        f"Joint edges  : {len(joint_graph['joint_edges'])}",
        flush=True,
    )
    print(f"Output       : {output_path}", flush=True)


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
        description="Generate GNG workspace graph"
    )

    parser.add_argument(
        "--input",
        type=Path,
        default=default_data_dir / "raw_workspace.npz",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=default_data_dir / "workspace_graph.npz",
    )

    args = parser.parse_args()

    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()

    if not input_path.is_file():
        raise FileNotFoundError(
            f"Workspace file not found: {input_path}"
        )

    config = load_configuration()

    generate_gng(
        config,
        input_path,
        output_path,
    )


if __name__ == "__main__":
    main()