"""Compare freshness budgets offline; never publish ROS or motion commands.

From the workspace root, with NumPy/MuJoCo available:
PYTHONPATH=src/planning:src/simulation python3 \
    src/planning/test/compare_human_timeouts.py > /tmp/cra_timeout_comparison.json

Each trial creates a cold planner on the same stationary synthetic capsule.
Capture ages are controlled inputs, not measurements of the live sensor.
"""

import argparse
import hashlib
import json
import platform
import random
import statistics
import time
from types import SimpleNamespace

import numpy as np

from planning.human_capsule_monitor import HumanCapsuleMonitor
from test_dynamic_replanning import DETOUR, DynamicReplanningTests, GOAL, ROOT, START


TIMEOUTS = (.5, 1., 2.)
CAPTURE_AGES = (0., .49, .75, 1.5, 2.1)
REAL_MONOTONIC = time.monotonic


def observed_message(capsules, captured_ns):
    def point(values):
        return SimpleNamespace(x=values[0], y=values[1], z=values[2])

    return SimpleNamespace(
        valid=True,
        header=SimpleNamespace(
            frame_id='base_link',
            stamp=SimpleNamespace(sec=captured_ns // 1_000_000_000,
                                  nanosec=captured_ns % 1_000_000_000)),
        capsules=[SimpleNamespace(body_id=c.body_id, segment_id=c.segment_id,
                                  start=point(c.start), end=point(c.end), radius=c.radius)
                  for c in capsules],
    )


def fresh_snapshot(case, manager, capsules, timeout, age):
    """Apply the selected budget at both monitoring and collision-check layers."""
    checker = manager.planner.collision_filter
    checker.human_data_timeout = timeout
    case.now.side_effect = REAL_MONOTONIC
    received = REAL_MONOTONIC()
    now_ros_ns = 1_000_000_000_000
    monitor = HumanCapsuleMonitor(data_timeout=timeout)
    accepted = monitor.update(
        observed_message(capsules, now_ros_ns - round(age * 1e9)), now_ros_ns)
    elapsed_ns = round((REAL_MONOTONIC() - received) * 1e9)
    snapshot = monitor.snapshot(now_ros_ns + elapsed_ns)
    manager.update_human_snapshot(snapshot)
    return accepted, snapshot, received


def detour_trial(timeout, age, repetition):
    case = DynamicReplanningTests()
    case.setUp()
    try:
        manager = case.make_manager(DETOUR, joints=START)
        accepted_input, snapshot, received = fresh_snapshot(
            case, manager, DETOUR.capsules, timeout, age)
        began = REAL_MONOTONIC()
        accepted_path = manager.planner.plan(START, GOAL)
        finished = REAL_MONOTONIC()
        path = manager.planner.current_path
        reason = next((line.split('NO_PATH: ', 1)[1]
                       for line in reversed(case.output.getvalue().splitlines())
                       if '[PathPlanner] NO_PATH: ' in line), None)
        return {
            'timeout_s': timeout, 'capture_age_input_s': age, 'repetition': repetition,
            'capture_age_at_plan_start_s': age + began - received,
            'capture_age_at_return_s': age + finished - received,
            'remaining_budget_at_plan_start_s': max(0., timeout - age - began + received),
            'wall_seconds': finished - began, 'input_accepted': bool(accepted_input),
            'snapshot_status': snapshot.status, 'path_accepted': bool(accepted_path),
            'failure': manager.planner.last_plan_failure, 'failure_detail': reason,
            'deadline_passed_at_return': age + finished - received > timeout,
            'waypoints': len(path) if path is not None else 0,
            'joint_path_length_rad': (float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                                      if path is not None else None),
            'path_sha256': (hashlib.sha256(np.asarray(path, dtype='<f8').tobytes()).hexdigest()
                            if path is not None else None),
        }
    finally:
        case.doCleanups()


def near_scene_trial(timeout):
    case = DynamicReplanningTests()
    case.setUp()
    try:
        manager, snapshot = case.margin_manager(clearance=.06)
        checker = manager.planner.collision_filter
        initial = checker.human_collision_report(manager.current_q)['clearance']
        ordinary_valid = checker.is_state_valid(manager.current_q)
        fresh_snapshot(case, manager, snapshot.capsules, timeout, 0.)
        began = REAL_MONOTONIC()
        path = manager.planner.find_human_margin_retreat(manager.current_q)
        duration = REAL_MONOTONIC() - began
        failure = manager.planner.last_human_retreat_failure
        # Endpoint geometry is checked separately with a frozen clock. This
        # inspection cannot turn a timed-out candidate into an accepted route.
        case.now.side_effect = None
        case.now.return_value = 100.
        manager.update_human_snapshot(snapshot)
        endpoint = (checker.human_collision_report(path[-1])['clearance']
                    if path is not None else None)
        return {'timeout_s': timeout, 'initial_clearance_m': initial,
                'ordinary_start_valid': bool(ordinary_valid), 'wall_seconds': duration,
                'retreat_found': path is not None, 'failure': failure,
                'endpoint_clearance_m': endpoint}
    finally:
        case.doCleanups()


def compare(repeats):
    trials = [(timeout, age, repeat) for repeat in range(repeats)
              for timeout in TIMEOUTS for age in CAPTURE_AGES]
    random.Random(42).shuffle(trials)
    rows = [detour_trial(*trial) for trial in trials]
    summary = []
    for age in CAPTURE_AGES:
        for timeout in TIMEOUTS:
            group = [r for r in rows if r['timeout_s'] == timeout
                     and r['capture_age_input_s'] == age]
            summary.append({
                'timeout_s': timeout, 'capture_age_input_s': age,
                'accepted': sum(r['path_accepted'] for r in group), 'trials': len(group),
                'mean_wall_seconds': statistics.mean(r['wall_seconds'] for r in group),
                'failures': sorted({r['failure'] for r in group if not r['path_accepted']}),
            })
    files = ['src/cra/config/robot.yaml',
             'src/cra/models/robot_model/cognitive_robotics_arm.xml',
             'src/planning/data/workspace_graph.npz',
             'src/planning/planning/path_planner.py',
             'src/planning/planning/algorithms/a_star.py',
             'src/planning/planning/algorithms/collision_filter.py',
             'src/planning/planning/human_capsule_monitor.py']
    return {
        'scope': 'Offline synthetic stationary geometry; geometric path acceptance, '
                 'not ROS transport or Motion execution. No live configuration is edited.',
        'python_version': platform.python_version(), 'repeats': repeats,
        'timeout_candidates_s': TIMEOUTS, 'controlled_capture_ages_s': CAPTURE_AGES,
        'scene': {'start': START.tolist(), 'goal': GOAL.tolist(),
                  'capsules': [vars(c) for c in DETOUR.capsules],
                  'human_safety_margin_m': .1},
        'source_sha256': {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in files},
        'summary': summary, 'detour_trials': rows,
        'fresh_near_scene_controls': [near_scene_trial(t) for t in TIMEOUTS],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=3)
    arguments = parser.parse_args()
    if arguments.repeats < 1:
        parser.error('--repeats must be positive')
    print(json.dumps(compare(arguments.repeats), indent=2))
