"""Offline XYZ-mask timing on the stored GNG; excludes ROS/rendering and robot checks."""

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import time

import numpy as np

from planning.visualization.workspace_graph_mask import WorkspaceGraphMask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    graph_path = root / 'src/planning/data/workspace_graph.npz'
    with np.load(graph_path, allow_pickle=False) as graph:
        positions, edges = graph['positions'], graph['edges']
    centers = positions[np.linspace(0, len(positions) - 1, 16, dtype=int)]
    capsules = [(p, p + [0., 0., .15], .05) for p in centers]
    boxes = [(positions[300], np.eye(3), [.05, .03, .04])]
    rows = []
    for name, selected, selected_boxes in (
            ('one_capsule', capsules[:1], []), ('16_capsules', capsules, []),
            ('16_capsules_and_box', capsules, boxes)):
        mask = WorkspaceGraphMask(positions, edges)
        elapsed, visible_updates = [], 0
        for index in range(50):
            shift = np.array([index * .0001, 0., 0.])
            shifted = [(a + shift, b + shift, radius) for a, b, radius in selected]
            began = time.perf_counter()
            result = mask.update(shifted, selected_boxes)
            elapsed.append((time.perf_counter() - began) * 1000.)
            visible_updates += result is not None
        rows.append(dict(scene=name, changed_geometry_updates=50,
                         visible_change_updates=visible_updates,
                         median_calculation_ms=statistics.median(elapsed),
                         max_calculation_ms=max(elapsed)))
        before = mask.calculations
        began = time.perf_counter()
        for _ in range(1000):
            assert mask.update(shifted, selected_boxes) is None
        rows[-1]['unchanged_call_mean_ms'] = (time.perf_counter() - began)
        rows[-1]['unchanged_extra_calculations'] = mask.calculations - before
        assert mask.calculations == before
    report = dict(scope=__doc__, nodes=len(positions), edges=len(edges), scenes=rows)
    files = [Path(__file__), graph_path,
             root / 'src/planning/planning/visualization/workspace_graph_mask.py',
             root / 'src/planning/planning/algorithms/human_collision.py']
    report['source_sha256'] = {str(p.relative_to(root)): hashlib.sha256(
        p.read_bytes()).hexdigest() for p in files}
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
