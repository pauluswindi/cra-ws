"""Compare bounded retreat queries on fixed native MuJoCo geometry.

Scenes are synthetic capsules, not reconstructed live camera observations.
Frozen observation time isolates algorithmic work; measured timings use the
real perf_counter. This is not a live latency or moving-human safety claim.
"""

import argparse
import ast
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import time
from unittest.mock import patch

import numpy as np

from benchmark_rrt_connect import FAR, ROOT, configure, geom_center, planner
from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from test_human_retreat_search import PlateauChecker, planner_for


def git_output(*arguments):
    return subprocess.check_output(
        ['git', '-c', 'safe.directory=' + str(ROOT), '-C', str(ROOT), *arguments], text=True)


def reference_method(revision):
    source = git_output('show', revision + ':src/planning/planning/path_planner.py')
    tree = ast.parse(source)
    method = next(f for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'PathPlanner'
                  for f in c.body if isinstance(f, ast.FunctionDef)
                  and f.name == 'find_human_margin_retreat')
    method_source = ast.get_source_segment(source, method)
    namespace = {'np': np}
    exec(compile(method_source, '<reference-retreat>', 'exec'), namespace)
    return namespace['find_human_margin_retreat'], method_source


def near_scene(checker, joints, geom, direction):
    center = geom_center(checker, joints, geom)
    low, high = 0., 1.5
    for _ in range(36):
        offset = (low + high) / 2.
        point = tuple(center + np.asarray(direction) * offset)
        snapshot = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 6, point, point, .06),))
        checker.set_human_capsules(snapshot)
        if checker.human_collision_report(joints)['clearance'] < .018:
            low = offset
        else:
            high = offset
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-revision', default='HEAD')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    original, reference_source = reference_method(args.reference_revision)
    reference_commit = git_output('rev-parse', args.reference_revision).strip()
    rows = []
    with patch('time.monotonic', return_value=100.):
        p = planner(FAR)
        checker = p.collision_filter
        positions = [
            ('link5_fixture', np.array([1.0831362616291125, -.7116884074291262,
                                       1.2431678147673046, -.15903829199563738,
                                       .3179308568765186]), 'link5_visual'),
            ('log_joint_pose_synthetic_capsule', np.array([
                1.62350120219812, -.8895556763787911, 1.926384895847176,
                -.1385219668203575, -1.2522302134801178]), 'link2_visual'),
        ]
        directions = [(1., 0., 0.), (-1., 0., 0.), (0., 1., 0.), (0., -1., 0.),
                      (0., 0., 1.), (0., 0., -1.)]
        for name, start, geom in positions:
            for direction in directions:
                snapshot = near_scene(checker, start, geom, direction)
                configure(checker, snapshot)
                initial = checker.human_margin_state(start)
                if initial is None or initial['status'] != 'NEAR':
                    rows.append(dict(scene=name, direction=direction, skipped='INVALID_SCENE'))
                    continue
                output = {}
                for mode in ('reference', 'current'):
                    trials = []
                    for _ in range(3):
                        configure(checker, snapshot)
                        queries = dict(states=0, transitions=0)
                        state_method = checker.human_margin_state
                        edge_method = checker.is_human_margin_transition_valid

                        def state(q):
                            queries['states'] += 1
                            return state_method(q)

                        def edge(a, b):
                            queries['transitions'] += 1
                            return edge_method(a, b)

                        with patch.object(checker, 'human_margin_state', side_effect=state), \
                                patch.object(checker, 'is_human_margin_transition_valid',
                                             side_effect=edge):
                            began = time.perf_counter()
                            path = (original(p, start) if mode == 'reference'
                                    else p.find_human_margin_retreat(start))
                            elapsed = time.perf_counter() - began
                        if path is not None:
                            assert edge_method(path[0], path[-1])
                            end = state_method(path[-1])
                            assert end['clearance'] - initial['clearance'] >= checker.safety_margin
                        trials.append(dict(elapsed_s=elapsed, **queries))
                    output[mode] = dict(
                        path=None if path is None else path.tolist(),
                        reason=p.last_human_retreat_failure,
                        median_s=statistics.median(t['elapsed_s'] for t in trials),
                        state_calls=trials[-1]['states'], edge_calls=trials[-1]['transitions'])
                    if mode == 'current':
                        output[mode]['statistics'] = p.last_human_retreat_statistics.copy()
                if output['reference']['path'] is not None:
                    np.testing.assert_array_equal(output['current']['path'],
                                                  output['reference']['path'])
                rows.append(dict(scene=name, direction=direction,
                                 initial_clearance_m=initial['clearance'], **output))
    plateau = {}
    for name, method in [('reference', original), ('current', type(p).find_human_margin_retreat)]:
        control = planner_for(PlateauChecker())
        path = method(control, np.zeros(5))
        plateau[name] = dict(path=None if path is None else path.tolist(),
                             reason=control.last_human_retreat_failure)
    files = [Path(__file__), ROOT / 'src/planning/planning/path_planner.py',
             ROOT / 'src/planning/planning/algorithms/collision_filter.py',
             ROOT / 'src/cra/config/robot.yaml']
    report = dict(scope=__doc__, reference_commit=reference_commit,
                  reference_method_sha256=hashlib.sha256(reference_source.encode()).hexdigest(),
                  native_scenes=rows, synthetic_dead_end=plateau,
                  source_sha256={
                      str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
                      for f in files})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    for row in rows:
        if 'skipped' in row:
            print(row)
            continue
        print(row['scene'], row['direction'],
              'edges', row['reference']['edge_calls'], '->', row['current']['edge_calls'],
              'ms', round(row['reference']['median_s'] * 1000., 2), '->',
              round(row['current']['median_s'] * 1000., 2),
              'path', row['current']['path'] is not None)
    print('dead_end', plateau)


if __name__ == '__main__':
    main()
