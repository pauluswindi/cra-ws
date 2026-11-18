"""Offline CRA geometry experiment, not ROS execution or a planner speed ranking.

Human observations are deliberately frozen for equal geometry; RRT's deadline
uses real perf_counter independently. Existing A* limits are unchanged (no new
wall deadline). The artificial two-node graph control is not the live GNG graph.
"""

import argparse
import contextlib
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import statistics
import time
from unittest.mock import patch

import numpy as np
import yaml

from planning.algorithms.a_star import AStarPlanner
from planning.algorithms.collision_filter import CollisionFilter
from planning.algorithms.rrt_connect import RRTConnectPlanner
from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from planning.path_planner import PathPlanner
from simulation.frame_transform import FrameTransform
from test_dynamic_replanning import FAR, GOAL, ROOT, START


MODEL = ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml'
CONFIG = ROOT / 'src/cra/config/robot.yaml'
GRAPH = ROOT / 'src/planning/data/workspace_graph.npz'
SETTINGS = yaml.safe_load(CONFIG.read_text())
HUMAN = SETTINGS['planning']['human_avoidance']


def configure(checker, snapshot):
    checker.human_safety_margin = float(HUMAN['safety_margin'])
    checker.human_data_timeout = float(HUMAN['data_timeout'])
    checker.enable_human_obstacles(True)
    checker.set_human_capsules(snapshot)


def planner(snapshot):
    with contextlib.redirect_stdout(io.StringIO()):
        result = PathPlanner(GRAPH, MODEL, CONFIG)
    configure(result.collision_filter, snapshot)
    return result


def geom_center(checker, joints, name):
    checker._set_joint_configuration(joints)
    base = checker.data.xpos[checker.base_id]
    rotation = checker.data.xmat[checker.base_id].reshape(3, 3)
    geom = checker.model.geom(name).id
    return FrameTransform.mujoco_base_to_robot_base(
        rotation.T @ (checker.data.geom_xpos[geom] - base))


def make_scenes():
    c = planner(FAR).collision_filter
    point = geom_center(c, (START + GOAL) / 2., 'link4_visual')
    blocked = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
        HumanCapsule(0, 0, tuple(point), tuple(point + [0., 0., .08]), .05),))
    retreat_start = np.array([1.0831362616291125, -.7116884074291262,
                              1.2431678147673046, -.15903829199563738, .3179308568765186])
    center = geom_center(c, retreat_start, 'link5_visual')
    low, high = 0., .5
    for _ in range(40):
        offset = (low + high) / 2.
        point = tuple(center + [offset, 0., 0.])
        close = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
            HumanCapsule(1, 6, point, point, .05),))
        c.set_human_capsules(close)
        if c.human_collision_report(retreat_start)['clearance'] < .03:
            low = offset
        else:
            high = offset
    return [
        ('far_human', FAR, START, GOAL),
        ('human_blocks_direct_segment', blocked, START, GOAL),
        ('artificial_graph_gap', blocked, START, GOAL),
        ('start_inside_5cm_margin', close, retreat_start, GOAL),
        ('expired_human', HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', FAR.capsules, 99.),
         START, GOAL),
    ]


def recertify(p, snapshot, path, start, goal):
    prepared = p.prepare_execution_path(path)
    np.testing.assert_array_equal(prepared[0], start)
    np.testing.assert_array_equal(prepared[-1], goal)
    assert np.abs(np.diff(prepared, axis=0)).max() <= p.max_segment_delta + 1e-12
    # New MuJoCo data, no geometry cache, and twice-finer edge sampling.
    c = CollisionFilter(MODEL, p.joint_names,
                        safety_margin=SETTINGS['planning']['obstacle_safety_margin'],
                        max_joint_step=.01, human_report_cache_size=0)
    configure(c, snapshot)
    assert c.is_path_valid(prepared)
    samples = []
    for a, b in zip(prepared[:-1], prepared[1:]):
        count = max(1, int(np.ceil(np.abs(b - a).max() / c.max_joint_step)))
        samples.extend(a + (b - a) * t for t in np.linspace(0., 1., count + 1))
    clearance = min(c.human_collision_report(q)['clearance'] for q in samples)
    assert clearance > c.human_safety_margin
    return {'prepared_waypoints': len(prepared), 'independent_edge_step_rad': .01,
            'min_sampled_human_clearance_m': clearance,
            'prepared_path': prepared.tolist()}


def trial(scene, method, seed, budget):
    name, snapshot, start, goal = scene
    p = planner(snapshot)
    c = p.collision_filter
    if name == 'artificial_graph_gap':
        # Deliberately omit all intermediate graph nodes. RRT must use new
        # configurations while all model, bounds and collision rules stay intact.
        p.joint_q = np.vstack((start, goal))
        p.planner = AStarPlanner(
            p.joint_q, np.empty((0, 2), dtype=int), np.empty(0),
            c, max_joint_delta=p.max_joint_delta)
    ready = c.human_data_ready()
    start_valid = c.is_state_valid(start)
    goal_valid = c.is_state_valid(goal)
    direct_valid = c.is_edge_valid(start, goal)
    if name in ('human_blocks_direct_segment', 'artificial_graph_gap'):
        assert start_valid and goal_valid and not direct_valid
    if name == 'start_inside_5cm_margin':
        assert not start_valid and 0. < c.human_collision_report(start)['clearance'] < .05
    if name == 'expired_human':
        assert not ready
    # Identical preflight for both methods, then cold per-snapshot caches.
    c._human_report_cache.clear()
    c.human_report_cache_hits = c.human_report_cache_misses = 0
    began = time.perf_counter()
    if method == 'rrt_connect':
        search = RRTConnectPlanner(c, time_budget=budget)
        path = search.plan(start, goal, seed=seed)
        reason, stats = search.last_plan_failure, search.last_statistics.copy()
        if name == 'artificial_graph_gap' and path is not None:
            assert stats['iterations'] > 0 and len(path) > 2
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            p.plan(start, goal)
        path = p.current_path
        reason, stats = p.last_plan_failure, {}
    elapsed = time.perf_counter() - began
    row = {'scene': name, 'method': method, 'seed': seed,
           'observation_ready': ready, 'start_valid': start_valid, 'goal_valid': goal_valid,
           'direct_edge_valid': direct_valid, 'path_found': path is not None,
           'reason': reason, 'search_wall_s': elapsed, 'search_statistics': stats,
           'path_length_rad': None, 'recertified': False}
    if path is not None:
        row['path_length_rad'] = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        row.update(recertify(p, snapshot, path, start, goal))
        row['recertified'] = True
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', type=int, default=8)
    parser.add_argument('--rrt-budget', type=float, default=.5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error('--seeds must be positive')
    rows = []
    with patch('time.monotonic', lambda: 100.):
        scenes = make_scenes()
        for scene in scenes:
            for seed in range(args.seeds):
                # Alternate order; each request gets a fresh model/checker/planner.
                methods = ('a_star', 'rrt_connect') if seed % 2 == 0 else ('rrt_connect', 'a_star')
                for method in methods:
                    rows.append(trial(scene, method, seed, args.rrt_budget))
            print('Completed', scene[0], flush=True)
    summaries = []
    for name, _, _, _ in scenes:
        for method in ('a_star', 'rrt_connect'):
            group = [r for r in rows if r['scene'] == name and r['method'] == method]
            lengths = [r['path_length_rad'] for r in group if r['path_found']]
            summaries.append({
                'scene': name, 'method': method, 'successes': sum(r['path_found'] for r in group),
                'trials': len(group), 'median_search_wall_s': statistics.median(
                    r['search_wall_s'] for r in group),
                'median_path_length_rad': statistics.median(lengths) if lengths else None,
                'reasons': sorted({r['reason'] for r in group}),
            })
    assert all(r['recertified'] for r in rows if r['path_found'])
    assert not any(r['path_found'] for r in rows if r['scene'] in (
        'start_inside_5cm_margin', 'expired_human'))
    assert not any(r['path_found'] for r in rows if r['scene'] == 'artificial_graph_gap'
                   and r['method'] == 'a_star')
    assert all(r['path_found'] for r in rows if r['scene'] == 'artificial_graph_gap'
               and r['method'] == 'rrt_connect')
    files = [MODEL, CONFIG, GRAPH, Path(__file__), ROOT / 'src/planning/planning/path_planner.py',
             ROOT / 'src/planning/planning/algorithms/rrt_connect.py',
             ROOT / 'src/planning/planning/algorithms/collision_filter.py',
             ROOT / 'src/planning/planning/algorithms/human_collision.py',
             ROOT / 'src/planning/planning/algorithms/a_star.py']
    report = {'scope': __doc__, 'rrt_budget_s': args.rrt_budget,
              'human_margin_m': HUMAN['safety_margin'], 'human_timeout_s': HUMAN['data_timeout'],
              'scenes': [{'name': name, 'snapshot': asdict(snapshot),
                          'start': start.tolist(), 'goal': goal.tolist()}
                         for name, snapshot, start, goal in scenes],
              'source_sha256': {
                  str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in files},
              'summary': summaries, 'trials': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
