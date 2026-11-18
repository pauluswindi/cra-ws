"""Pair baseline and preferred RRT paths under frozen geometric perturbations.

This tests one synthetic capsule scene, not live sensing, filtering or robot
execution. Each candidate is recertified independently. Timing includes the
extra search/selection cost; this is not an equal-budget planner speed race.
"""

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import statistics
import time
from unittest.mock import patch

import numpy as np

from benchmark_rrt_connect import (
    CONFIG, GRAPH, HUMAN, MODEL, ROOT, SETTINGS, configure, make_scenes, planner, recertify,
)
from planning.algorithms.clearance_preference import ClearancePathPreference
from planning.algorithms.collision_filter import CollisionFilter
from planning.algorithms.rrt_connect import RRTConnectPlanner


def perturbations():
    yield np.zeros(3)
    for distance in (.001, .003, .005):
        for axis in range(3):
            for sign in (-1., 1.):
                offset = np.zeros(3)
                offset[axis] = sign * distance
                yield offset


def shifted(snapshot, offset):
    return replace(snapshot, capsules=tuple(replace(
        capsule, start=tuple(np.asarray(capsule.start) + offset),
        end=tuple(np.asarray(capsule.end) + offset)) for capsule in snapshot.capsules))


def check_perturbations(names, snapshot, path):
    checker = CollisionFilter(
        MODEL, names, safety_margin=SETTINGS['planning']['obstacle_safety_margin'],
        human_safety_margin=HUMAN['safety_margin'], max_joint_step=.01,
        human_data_timeout=HUMAN['data_timeout'], human_report_cache_size=0)
    rows = []
    for offset in perturbations():
        configure(checker, shifted(snapshot, offset))
        valid = checker.is_path_valid(path)
        failure = checker.last_path_validation_failure
        details = {} if failure is None else failure['failure']
        # Edge diagnostics wrap the rejected state; waypoint diagnostics do not.
        state_failure = details.get('state') or details
        rows.append({'offset_m': offset.tolist(), 'magnitude_m': float(np.linalg.norm(offset)),
                     'valid': bool(valid),
                     'failure_reason': None if valid else state_failure.get('reason'),
                     'failure': failure})
    return rows


def refined_clearance(names, snapshot, path):
    checker = CollisionFilter(
        MODEL, names, safety_margin=SETTINGS['planning']['obstacle_safety_margin'],
        human_safety_margin=HUMAN['safety_margin'], max_joint_step=.01,
        human_data_timeout=HUMAN['data_timeout'], human_report_cache_size=0)
    configure(checker, snapshot)
    minimum = float('inf')
    for a, b in zip(path[:-1], path[1:]):
        count = max(1, int(np.ceil(np.abs(b - a).max() / .01)))
        for fraction in np.linspace(0., 1., count + 1):
            point = a + (b - a) * fraction
            assert checker.is_state_valid(point)
            report = checker._human_collision_after_forward(include_pairs=True)
            minimum = min(minimum, report['clearance'])
    return minimum


def trial(scene, seed, preference, selection_budget, search_budget):
    _, snapshot, start, goal = scene
    p = planner(snapshot)
    search = RRTConnectPlanner(p.collision_filter)
    began = time.perf_counter()
    baseline = search.plan(start, goal, seed=seed)
    row = {'seed': seed, 'baseline_reason': search.last_plan_failure,
           'baseline_search_s': time.perf_counter() - began, 'paths': {}}
    if baseline is None:
        return row
    selector = ClearancePathPreference(
        p.collision_filter, preferred_clearance=preference,
        time_budget=selection_budget, search_budget=search_budget)
    selected = selector.improve(baseline, seed=seed)
    row['preference'] = selector.last_statistics.copy()
    for name, path in [('baseline', baseline), ('preferred', selected)]:
        if path is None:
            continue
        data = recertify(p, snapshot, path, start, goal)
        data['min_fast_lower_bound_m'] = data['min_sampled_human_clearance_m']
        data['min_sampled_human_clearance_m'] = refined_clearance(
            p.joint_names, snapshot, np.asarray(data['prepared_path']))
        data['length_rad'] = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        data['perturbations'] = check_perturbations(
            p.joint_names, snapshot, np.asarray(data['prepared_path']))
        row['paths'][name] = data
    if selected is not None:
        assert row['paths']['preferred']['length_rad'] <= (
            1.25 * row['paths']['baseline']['length_rad'] + 1e-12)
        assert row['paths']['preferred']['min_sampled_human_clearance_m'] >= (
            row['paths']['baseline']['min_sampled_human_clearance_m'] - 1e-9)
    assert p.collision_filter.human_safety_margin == .05
    print('seed', seed, 'selection', row['preference']['selection'],
          'candidate', row['preference']['candidate_reason'], flush=True)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', type=int, default=8)
    parser.add_argument('--preferred-clearance', type=float, default=.07)
    parser.add_argument('--selection-budget', type=float, default=2.)
    parser.add_argument('--search-budget', type=float, default=.8)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error('--seeds must be positive')
    with patch('time.monotonic', lambda: 100.):
        scene = make_scenes()[1]
        rows = [trial(scene, seed, args.preferred_clearance,
                      args.selection_budget, args.search_budget) for seed in range(args.seeds)]
    summaries = {}
    for name in ('baseline', 'preferred'):
        paths = [row['paths'][name] for row in rows if name in row['paths']]
        perturbation_results = {}
        for magnitude in (0., .001, .003, .005):
            checks = [check for path in paths for check in path['perturbations']
                      if abs(check['magnitude_m'] - magnitude) < 1e-12]
            perturbation_results[str(magnitude)] = {
                'valid': sum(check['valid'] for check in checks), 'total': len(checks)}
        summaries[name] = {
            'returned_paths': len(paths),
            'median_clearance_m': statistics.median(
                p['min_sampled_human_clearance_m'] for p in paths) if paths else None,
            'median_length_rad': (
                statistics.median(p['length_rad'] for p in paths) if paths else None),
            'perturbations': perturbation_results,
        }
    summaries['clearer_selected'] = sum(
        row.get('preference', {}).get('selection') == 'CLEARER_PATH_SELECTED' for row in rows)
    times = [row['preference']['elapsed_s'] for row in rows if 'preference' in row]
    summaries['median_preference_overhead_s'] = statistics.median(times) if times else None
    files = [Path(__file__), MODEL, CONFIG, GRAPH,
             ROOT / 'src/planning/test/benchmark_rrt_connect.py',
             ROOT / 'src/planning/planning/algorithms/rrt_connect.py',
             ROOT / 'src/planning/planning/algorithms/clearance_preference.py',
             ROOT / 'src/planning/planning/algorithms/collision_filter.py',
             ROOT / 'src/planning/planning/algorithms/human_collision.py']
    report = {
        'scope': __doc__, 'hard_margin_m': HUMAN['safety_margin'],
        'requested_preference_m': args.preferred_clearance, 'max_length_ratio': 1.25,
        'selection_budget_s': args.selection_budget,
        'candidate_search_budget_s': args.search_budget,
        'snapshot': {'capsules': [
            {'start': c.start, 'end': c.end, 'radius': c.radius} for c in scene[1].capsules]},
        'summary': summaries, 'trials': rows,
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in files},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
