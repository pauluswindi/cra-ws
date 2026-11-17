"""Compare exact geometry memoization on cold, frozen-snapshot CRA searches.

This is an offline geometry/CPU comparison, not a live human or Motion trial.
Freshness is deliberately frozen for equal work; timeout behavior is tested
separately. Each trial creates a new planner, without warm A* or geometry caches.
"""

import argparse
import hashlib
import json
from pathlib import Path
import random
import statistics
import time
from unittest.mock import patch

import numpy as np

from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from test_dynamic_replanning import DynamicReplanningTests, DETOUR, GOAL, ROOT, START


def trial(scene, capacity):
    case = DynamicReplanningTests()
    case.setUp()
    try:
        if scene == 'margin_retreat':
            manager, _ = case.margin_manager(clearance=.06)
        else:
            snapshot = DETOUR
            if scene == 'eight_capsules':
                snapshot = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', DETOUR.capsules + tuple(
                    HumanCapsule(0, i, (10. + i, 10., 10.), (10. + i, 10., 11.), .05)
                    for i in range(1, 8)))
            manager = case.make_manager(snapshot, joints=START)
        planner = manager.planner
        checker = planner.collision_filter
        checker.human_report_cache_size = capacity
        checker.human_data_timeout = 2.
        checker._human_report_cache.clear()
        checker.human_report_cache_hits = checker.human_report_cache_misses = 0
        # A plain clock callable avoids timing unittest.mock's bookkeeping.
        with patch('time.monotonic', lambda: 100.):
            began = time.perf_counter()
            if scene == 'margin_retreat':
                path = planner.find_human_margin_retreat(manager.current_q)
                failure = planner.last_human_retreat_failure
            else:
                planner.plan(START, GOAL)
                path = planner.current_path
                failure = planner.last_plan_failure
            duration = time.perf_counter() - began
            row = {'scene': scene, 'cache_capacity': capacity, 'wall_seconds': duration,
                   'path_found': path is not None, 'failure': failure,
                   'geometry_hits': checker.human_report_cache_hits,
                   'geometry_computations': checker.human_report_cache_misses,
                   'cache_entries': len(checker._human_report_cache),
                   'path_sha256': None, 'waypoints': 0}
            if path is not None:
                row['path_sha256'] = hashlib.sha256(
                    np.asarray(path, dtype='<f8').tobytes()).hexdigest()
                row['waypoints'] = len(path)
                # Independently certify the result with memoization disabled.
                checker.human_report_cache_size = 0
                if scene == 'margin_retreat':
                    assert checker.is_human_margin_transition_valid(path[0], path[-1])
                else:
                    assert checker.is_path_valid(path)
        return row
    finally:
        case.doCleanups()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=7)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    jobs = [(scene, capacity) for scene in ['detour', 'eight_capsules', 'margin_retreat']
            for _ in range(args.repeats) for capacity in [0, 256]]
    random.Random(42).shuffle(jobs)
    rows = [trial(*job) for job in jobs]
    summaries = []
    for scene in sorted({r['scene'] for r in rows}):
        group = [r for r in rows if r['scene'] == scene]
        assert len({(r['path_sha256'], r['failure']) for r in group}) == 1
        variants = {}
        for capacity in [0, 256]:
            variant = [r for r in group if r['cache_capacity'] == capacity]
            variants[str(capacity)] = {
                'median_wall_seconds': statistics.median(r['wall_seconds'] for r in variant),
                'geometry_computations': sorted({r['geometry_computations'] for r in variant}),
                'geometry_hits': sorted({r['geometry_hits'] for r in variant}),
                'path_found': all(r['path_found'] for r in variant),
                'failure': sorted({r['failure'] for r in variant}),
            }
        speedup = (variants['0']['median_wall_seconds'] /
                   variants['256']['median_wall_seconds'])
        summaries.append({'scene': scene, 'variants': variants, 'median_speedup': speedup})
    files = ['src/cra/config/robot.yaml', 'src/planning/data/workspace_graph.npz',
             'src/cra/models/robot_model/cognitive_robotics_arm.xml',
             'src/planning/planning/algorithms/collision_filter.py',
             'src/planning/planning/algorithms/human_collision.py',
             'src/planning/planning/algorithms/a_star.py',
             'src/planning/planning/path_planner.py',
             'src/planning/test/benchmark_collision_memoization.py']
    report = {'scope': __doc__, 'repeats_per_variant': args.repeats,
              'source_sha256': {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
                                for p in files},
              'same_path_and_failure_in_all_variants': True,
              'all_returned_paths_recertified_without_cache': True,
              'summary': summaries, 'trials': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
