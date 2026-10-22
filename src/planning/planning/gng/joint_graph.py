import numpy as np


def build_joint_graph(
    prototypes,
    gng_edges,
    workspace_eof,
    workspace_q,
    max_candidates=4,
    nearest_pool=64,
    max_joint_delta=0.6,
    initial_q=None,
    local_connections=8,
):
    prototypes = np.asarray(prototypes, dtype=np.float64)
    eof = np.asarray(workspace_eof, dtype=np.float64)
    q = np.asarray(workspace_q, dtype=np.float64)
    gng_edges = np.asarray(gng_edges, dtype=np.int64).reshape(-1, 2)

    if (
        prototypes.ndim != 2
        or prototypes.shape[1] != 3
        or len(prototypes) == 0
    ):
        raise ValueError("Invalid GNG prototypes")

    if (
        eof.ndim != 2
        or eof.shape[1] != 3
        or len(eof) == 0
        or q.shape != (len(eof), 5)
    ):
        raise ValueError("Invalid workspace data")

    if not all(
        np.isfinite(array).all()
        for array in (prototypes, eof, q)
    ):
        raise ValueError("Non-finite workspace data")

    if len(gng_edges) and (
        gng_edges.min() < 0
        or gng_edges.max() >= len(prototypes)
    ):
        raise ValueError("Invalid GNG edge indices")

    if (
        max_candidates < 1
        or nearest_pool < 1
        or local_connections < 0
        or not np.isfinite(max_joint_delta)
        or max_joint_delta <= 0
    ):
        raise ValueError("Invalid graph parameters")

    if initial_q is not None:
        initial_q = np.asarray(initial_q, dtype=np.float64)

        if (
            initial_q.shape != (5,)
            or not np.isfinite(initial_q).all()
        ):
            raise ValueError("initial_q must contain five finite joints")

    num_nodes = len(prototypes)

    neighbors = [set() for _ in range(num_nodes)]

    for a, b in gng_edges:
        if a != b:
            neighbors[a].add(int(b))
            neighbors[b].add(int(a))

    # Assign each workspace sample to its nearest GNG prototype.
    memberships = np.empty(len(eof), dtype=np.int32)
    prototype_norm = np.sum(prototypes ** 2, axis=1)

    for start in range(0, len(eof), 256):
        block = eof[start:start + 256]

        distances = (
            np.sum(block ** 2, axis=1)[:, None]
            + prototype_norm
            - 2.0 * block @ prototypes.T
        )

        memberships[start:start + len(block)] = np.argmin(
            distances,
            axis=1,
        )

    pools = []

    for node, position in enumerate(prototypes):
        indices = np.flatnonzero(memberships == node)

        if len(indices) == 0:
            nearest = np.argmin(
                np.sum((eof - position) ** 2, axis=1)
            )
            indices = np.array([nearest], dtype=np.int64)

        distances = np.sum(
            (eof[indices] - position) ** 2,
            axis=1,
        )

        count = min(nearest_pool, len(indices))

        order = np.argsort(
            distances,
            kind="stable",
        )[:count]

        pools.append(indices[order])

    selected = [[int(pool[0])] for pool in pools]

    def remaining_samples(node):
        pool = pools[node]

        return pool[
            ~np.isin(pool, selected[node])
        ]

    def select_bridge(node):
        remaining = remaining_samples(node)

        if len(remaining) == 0 or not neighbors[node]:
            return None

        scores = np.zeros(len(remaining), dtype=np.int32)
        distances = np.full(len(remaining), np.inf)

        for other in neighbors[node]:
            other_q = q[selected[other]]

            deltas = np.max(
                np.abs(
                    q[remaining, None, :]
                    - other_q[None, :, :]
                ),
                axis=2,
            )

            nearest = deltas.min(axis=1)

            scores += nearest <= max_joint_delta
            distances = np.minimum(distances, nearest)

        if scores.max() == 0:
            return None

        ranking = np.lexsort((distances, -scores))

        return int(remaining[ranking[0]])

    def select_diverse(node):
        remaining = remaining_samples(node)

        if len(remaining) == 0:
            return None

        selected_q = q[selected[node]]

        distances = np.linalg.norm(
            q[remaining, None, :]
            - selected_q[None, :, :],
            axis=2,
        )

        scores = distances.min(axis=1)
        best = int(np.argmax(scores))

        if scores[best] <= 1e-5:
            return None

        return int(remaining[best])

    # Preserve GNG connectivity and posture diversity.
    for round_index in range(1, max_candidates):
        for node in range(num_nodes):
            if len(selected[node]) >= max_candidates:
                continue

            if round_index == 2:
                candidate = select_diverse(node)
            else:
                candidate = select_bridge(node)

            if candidate is None:
                candidate = select_diverse(node)

            if candidate is not None:
                selected[node].append(candidate)

    # Preserve a workspace sample near the actual robot start.
    initial_sample = None

    if initial_q is not None:
        distances = np.max(
            np.abs(q - initial_q),
            axis=1,
        )

        initial_sample = int(np.argmin(distances))
        initial_distance = float(distances[initial_sample])

        if initial_distance > max_joint_delta:
            raise ValueError(
                "Workspace does not cover initial_q: "
                f"nearest max joint delta is "
                f"{initial_distance:.3f} rad, "
                f"limit is {max_joint_delta:.3f} rad"
            )

        node = int(memberships[initial_sample])

        if initial_sample not in selected[node]:
            if len(selected[node]) < max_candidates:
                selected[node].append(initial_sample)
            else:
                selected[node][-1] = initial_sample

    sample_indices = np.asarray(
        [
            index
            for group in selected
            for index in group
        ],
        dtype=np.int64,
    )

    node_ids = np.asarray(
        [
            node
            for node, group in enumerate(selected)
            for _ in group
        ],
        dtype=np.int32,
    )

    joint_q = q[sample_indices]

    offsets = np.cumsum(
        [0] + [len(group) for group in selected]
    )

    edge_pairs = set()

    def connect(first_states, second_states):
        for a in first_states:
            for b in second_states:
                if a == b:
                    continue

                delta = np.max(
                    np.abs(joint_q[a] - joint_q[b])
                )

                if delta <= max_joint_delta:
                    edge_pairs.add(
                        (min(a, b), max(a, b))
                    )

    # Connect configurations associated with the same GNG node.
    for node in range(num_nodes):
        states = range(
            offsets[node],
            offsets[node + 1],
        )

        connect(states, states)

    # Connect configurations associated with neighboring GNG nodes.
    for a, b in gng_edges:
        connect(
            range(offsets[a], offsets[a + 1]),
            range(offsets[b], offsets[b + 1]),
        )

    # Connect the preserved start sample to nearby joint states.
    if initial_sample is not None and local_connections > 0:
        start_index = int(
            np.flatnonzero(
                sample_indices == initial_sample
            )[0]
        )

        delta = np.max(
            np.abs(joint_q - joint_q[start_index]),
            axis=1,
        )

        candidates = np.flatnonzero(
            (delta <= max_joint_delta)
            & (np.arange(len(joint_q)) != start_index)
        )

        distances = np.linalg.norm(
            joint_q[candidates] - joint_q[start_index],
            axis=1,
        )

        order = np.argsort(
            distances,
            kind="stable",
        )[:local_connections]

        for index in candidates[order]:
            a, b = sorted((start_index, int(index)))
            edge_pairs.add((a, b))

    edges = np.asarray(
        sorted(edge_pairs),
        dtype=np.int64,
    ).reshape(-1, 2)

    if len(edges):
        costs = np.linalg.norm(
            joint_q[edges[:, 0]]
            - joint_q[edges[:, 1]],
            axis=1,
        )
    else:
        costs = np.empty(0, dtype=np.float64)

    return {
        "joint_q": joint_q.astype(np.float32),
        "joint_eof": eof[sample_indices].astype(np.float32),
        "joint_node_ids": node_ids,
        "joint_sample_indices": sample_indices,
        "joint_edges": edges,
        "joint_costs": costs.astype(np.float32),
    }