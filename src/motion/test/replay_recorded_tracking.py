#!/usr/bin/env python3
"""Diagnose recorded tracking with native Motion and headless MuJoCo.

No ROS node, publisher, viewer, controller tuning or model changes are used.
The C++ bridge compiles the production controller, trajectory and guards.
Simulation invokes the production Python control methods with stub ROS imports.
Wall/physics schedules are explicit experimental inputs, not live timing claims.
"""

import argparse
from bisect import bisect_right
import csv
import ctypes
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[3]
TEST = Path(__file__).resolve().parent
MONITOR = ['NORMAL', 'GOAL_TIMEOUT', 'STALLED', 'TRACKING_ERROR', 'INVALID_INPUT']
CONTROLLER = ['IDLE', 'RUNNING', 'COMPLETED', 'ERROR']


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compile_bridge(directory):
    """Build current source, avoiding stale installed controller libraries."""
    package = ROOT / 'src/motion'
    sources = [TEST / 'tracking_replay_bridge.cpp'] + [package / 'src' / name for name in [
        'controllers/joint_controller.cpp', 'controllers/execution_monitor.cpp',
        'modules/motion_guard.cpp', 'trajectory/trajectory_generator.cpp',
        'trajectory/motion_profile.cpp', 'kinematics/robot_kinematics.cpp',
        'kinematics/jacobian.cpp']]
    mujoco_root = Path(mujoco.__file__).parent
    libraries = sorted(mujoco_root.glob('libmujoco.so.*'))
    if len(libraries) != 1:
        raise RuntimeError('Expected one MuJoCo shared library')
    output = Path(directory) / 'tracking_bridge.so'
    command = ['g++', '-std=c++20', '-O2', '-shared', '-fPIC', '-Wall', '-Wextra',
               '-I' + str(package / 'include'), '-I/usr/include/eigen3',
               '-I' + str(mujoco_root / 'include')]
    for name in ['trajectory_msgs', 'builtin_interfaces', 'std_msgs',
                 'rosidl_runtime_cpp', 'rosidl_runtime_c',
                 'rosidl_typesupport_interface', 'rcutils']:
        command.append('-I/opt/ros/humble/include/' + name)
    command += [str(p) for p in sources]
    command += [str(libraries[0]), '-Wl,-rpath,' + str(mujoco_root), '-o', str(output)]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(error.stderr) from error
    library = ctypes.CDLL(str(output))
    pointer = ctypes.POINTER(ctypes.c_double)
    library.tracking_create.argtypes = [
        ctypes.c_char_p, pointer, ctypes.c_char_p, pointer, ctypes.c_int,
        pointer, ctypes.c_char_p, ctypes.c_int]
    library.tracking_create.restype = ctypes.c_void_p
    library.tracking_step.argtypes = [ctypes.c_void_p, ctypes.c_double, pointer, pointer]
    library.tracking_step.restype = ctypes.c_int
    library.tracking_destroy.argtypes = [ctypes.c_void_p]
    library.tracking_destroy.restype = None
    return library, sources


def pointer(values):
    return values.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


class NativeController:
    def __init__(self, library, config, fixture, model_path):
        self.library = library
        joint = config['motion']['joint']
        monitor = config['motion']['monitoring']
        parameters = np.array([
            joint['gain'], joint['speed'], joint['tolerance'],
            joint['start_tolerance'], joint['max_segment_delta'],
            config['motion']['cartesian']['speed'], config['trajectory']['distance_step'],
            monitor['goal_timeout'], monitor['stall_timeout'],
            monitor['tracking_error_limit'], monitor['tracking_error_timeout']], dtype=float)
        path = np.array([p['positions'] for p in fixture['command']['points']], dtype=float)
        self.initial = np.array(fixture['initial_feedback']['positions'], dtype=float)
        error = ctypes.create_string_buffer(2048)
        self.handle = library.tracking_create(
            str(model_path).encode(), pointer(parameters),
            config['trajectory']['curve'].encode(), pointer(path), len(path),
            pointer(self.initial), error, len(error))
        if not self.handle:
            raise RuntimeError(error.value.decode())

    def step(self, elapsed, feedback):
        feedback = np.ascontiguousarray(feedback, dtype=float)
        output = np.zeros(23)
        if self.library.tracking_step(self.handle, elapsed, pointer(feedback),
                                      pointer(output)) != 0:
            raise RuntimeError('Native controller/guard failed')
        return output

    def close(self):
        self.library.tracking_destroy(self.handle)


def simulation_robot(fixture, model_path):
    # Reuse the existing test loader: only ROS imports are stubs; physics and
    # RobotSimulation.update_control/velocity_callback remain production code.
    spec = importlib.util.spec_from_file_location(
        'cra_hold_imports', ROOT / 'src/simulation/test/test_hold_control.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cls = module.simulation_class()
    robot = cls.__new__(cls)
    robot.model = mujoco.MjModel.from_xml_path(str(model_path))
    robot.data = mujoco.MjData(robot.model)
    joints = [robot.model.joint(n).id for n in fixture['command']['names']]
    robot.qpos_ids = robot.model.jnt_qposadr[joints]
    robot.qvel_ids = robot.model.jnt_dofadr[joints]
    robot.actuator_ids = [next(i for i in range(robot.model.nu)
                               if robot.model.actuator_trnid[i, 0] == j) for j in joints]
    robot.q_lower = robot.model.jnt_range[joints, 0].copy()
    robot.q_upper = robot.model.jnt_range[joints, 1].copy()
    robot.base_id = robot.model.body('base_link').id
    initial = fixture['initial_feedback']
    robot.data.qpos[robot.qpos_ids] = initial['positions']
    robot.data.qvel[robot.qvel_ids] = initial['velocities']
    mujoco.mj_forward(robot.model, robot.data)
    target = fixture['target']
    target_body = robot.model.body('target').id
    robot.data.mocap_pos[robot.model.body_mocapid[target_body]] = (
        robot.robot_base_to_world(target['position']))
    # Recorded obstacle markers are DELETE (both obstacles are parked at their
    # XML defaults). Refuse a different scene instead of silently omitting it.
    if any(m['action'] != 2 for m in fixture['obstacles']['markers']):
        raise ValueError('This fixture loader requires parked obstacle markers')
    mujoco.mj_forward(robot.model, robot.data)
    robot.q_des = robot.hold_position_target()
    robot.command = np.zeros(5)
    robot.max_velocity = .5
    robot.command_timeout = .2
    robot.holding_position = True
    robot.last_command = None
    robot.last_debug_time = 1e20
    robot.pending_target = None
    robot.pending_obstacles = {}
    robot.dt = robot.model.opt.timestep
    robot.get_logger = lambda: SimpleNamespace(error=lambda *args: None)
    return robot


def physics_events(fixture, dt, end, ratio=None):
    if ratio is not None:
        for index in range(int(end * ratio / dt) + 1):
            yield index * dt / ratio, (index + 1) % 5 == 0
        return
    # Interpolate the five fixed physics steps between successive recorded
    # feedback stamps. The exact step/callback schedule was not recorded.
    times = [(r['stamp_ns'] - fixture['controller_start_ns']) / 1e9
             for r in fixture['feedback']]
    interval = float(np.median(np.diff(times)))
    while times[-1] < end:
        times.append(times[-1] + interval)
    for left, right in zip(times, times[1:]):
        for index in range(1, 6):
            wall = left + (right - left) * index / 5
            if 0 <= wall <= end:
                yield wall, index == 5


def row(elapsed, output, feedback, robot=None, feedback_age=0.):
    result = {'wall_s': elapsed, 'controller': CONTROLLER[int(output[20])],
              'monitor': MONITOR[int(output[21])], 'feedback_age_s': feedback_age}
    vectors = {'reference': output[:5], 'reference_velocity': output[5:10],
               'command': output[10:15], 'filtered': output[15:20],
               'feedback': feedback, 'error': output[:5] - feedback}
    if robot is not None:
        result.update(physics_s=float(robot.data.time), contacts=int(robot.data.ncon))
        vectors.update(actual=robot.joint_positions(),
                       actual_velocity=robot.data.qvel[robot.qvel_ids],
                       actuator_target=robot.q_des,
                       actuator_force=robot.data.qfrc_actuator[robot.qvel_ids],
                       bias_force=robot.data.qfrc_bias[robot.qvel_ids])
    for name, values in vectors.items():
        result.update({f'{name}_{i + 1}': float(v) for i, v in enumerate(values)})
    return result


def summarize(rows, duration):
    last = rows[-1]
    errors = np.array([[r[f'error_{i}'] for i in range(1, 6)] for r in rows])
    fault = last['monitor'] if last['monitor'] != 'NORMAL' else None
    joint = int(np.argmax(np.abs(errors[-1]))) + 1
    result = {'controller': last['controller'], 'fault': fault,
              'end_wall_s': last['wall_s'], 'trajectory_duration_s': duration,
              'peak_error_rad': np.max(np.abs(errors), axis=0).tolist(),
              'end_error_rad': errors[-1].tolist(),
              'max_feedback_age_s': max(r['feedback_age_s'] for r in rows)}
    result['saturation_ticks_by_joint'] = [
        sum(abs(r[f'command_{i}'] - r[f'filtered_{i}']) > 1e-9 for r in rows)
        for i in range(1, 6)]
    if fault:
        result['fault_detail'] = {name: last[f'{name}_{joint}'] for name in [
            'reference', 'feedback', 'error', 'reference_velocity', 'command', 'filtered']}
        result['fault_detail']['max_error_joint'] = joint
    if 'contacts' in last:
        result['max_contacts'] = max(r['contacts'] for r in rows)
        result['end_physics_s'] = last['physics_s']
    return result


def run_case(library, config, fixture, model, name, ratio=None):
    controller = NativeController(library, config, fixture, model)
    rows = []
    try:
        if name == 'recorded_feedback':
            stamps = [r['stamp_ns'] for r in fixture['feedback']]
            duration = (fixture['feedback'][-1]['stamp_ns'] -
                        fixture['controller_start_ns']) / 1e9
            for index in range(int(duration / .005) + 1):
                elapsed = index * .005
                ns = fixture['controller_start_ns'] + round(elapsed * 1e9)
                sample = fixture['feedback'][max(0, bisect_right(stamps, ns) - 1)]
                feedback = np.array(sample['positions'])
                output = controller.step(elapsed, feedback)
                rows.append(row(elapsed, output, feedback,
                                feedback_age=(ns - sample['stamp_ns']) / 1e9))
                if output[21] != 0 or output[20] != 1:
                    break
        else:
            with patch('time.monotonic', return_value=100.) as clock:
                robot = simulation_robot(fixture, model)
                tracking = config['motion'].get('tracking', {})
                robot.max_velocity = tracking.get('max_joint_velocity', .5)
                robot.command_timeout = tracking.get('command_timeout', .2)
                output = controller.step(0., controller.initial)
                horizon = float(output[22]) + config['motion']['monitoring']['goal_timeout'] + 1
                events = iter(physics_events(fixture, robot.dt, horizon, ratio))
                event = next(events)
                feedback = robot.joint_positions()
                feedback_time = 0.
                for index in range(int(horizon / .005) + 1):
                    elapsed = index * .005
                    # Physics events before this Motion tick use the previously
                    # published command. Ties dispatch Motion first.
                    while event[0] < elapsed - 1e-12:
                        clock.return_value = 100. + event[0]
                        robot.update_control()
                        mujoco.mj_step(robot.model, robot.data)
                        if event[1]:
                            feedback = robot.joint_positions()
                            feedback_time = event[0]
                        event = next(events, (float('inf'), False))
                    clock.return_value = 100. + elapsed
                    output = controller.step(elapsed, feedback)
                    rows.append(row(elapsed, output, feedback, robot,
                                    elapsed - feedback_time))
                    terminal = output[21] != 0 or output[20] != 1
                    velocity = np.zeros(5) if terminal else output[15:20]
                    robot.velocity_callback(SimpleNamespace(data=velocity))
                    if terminal:
                        break
        return summarize(rows, float(output[22])), rows
    finally:
        controller.close()


def timing_evidence(fixture, dt):
    samples = [r for r in fixture['feedback'] if
               fixture['controller_start_ns'] <= r['stamp_ns'] <=
               fixture['logged_fault']['stamp_ns']]
    intervals = np.diff([r['stamp_ns'] for r in samples]) / 1e9
    estimates = []
    for left, right in zip(samples, samples[1:]):
        mean_v = (np.array(left['velocities']) + right['velocities']) / 2
        dq = np.array(right['positions']) - left['positions']
        if np.dot(mean_v, mean_v) > .02:
            sim_dt = float(np.dot(mean_v, dq) / np.dot(mean_v, mean_v))
            estimates.append([sim_dt, sim_dt / ((right['stamp_ns'] -
                                                left['stamp_ns']) / 1e9)])
    elapsed = (samples[-1]['stamp_ns'] - samples[0]['stamp_ns']) / 1e9
    return {'feedback_rows_during_execution': len(samples),
            'wall_span_s': elapsed,
            'feedback_interval_median_s': float(np.median(intervals)),
            'physics_step_s': dt, 'source_feedback_stride': 5,
            'count_inferred_physics_span_s': (len(samples) - 1) * 5 * dt,
            'count_inferred_physics_to_wall_ratio': (len(samples) - 1) * 5 * dt / elapsed,
            'velocity_inferred_physics_interval_median_s': float(np.median(estimates, axis=0)[0]),
            'velocity_inferred_physics_to_wall_median': float(np.median(estimates, axis=0)[1]),
            'limitation': 'No physics clock was recorded. Step-count inference assumes no '
                          'feedback loss; independent dq/qvel estimates cross-check it.'}


def comparative_checks(result, fixture, config):
    """Check the causal comparison for this recorded fault, not all trajectories."""
    cases = result['cases']
    measured = cases['recorded_feedback']
    original_elapsed = (fixture['logged_fault']['stamp_ns'] -
                        fixture['controller_start_ns']) / 1e9
    realtime = cases['physics_realtime']
    slow = [cases[name] for name in ['physics_half_speed', 'physics_recorded_cadence']]
    return {
        'native_feedback_reproduces_recorded_fault': (
            measured['fault'] == fixture['logged_fault']['state'] and
            measured.get('fault_detail', {}).get('max_error_joint') ==
            fixture['logged_fault']['joint']),
        'native_feedback_fault_time_matches_within_50ms': (
            abs(measured['end_wall_s'] - original_elapsed) < .05),
        'native_feedback_fault_error_matches_within_01rad': (
            abs(measured.get('fault_detail', {}).get('error', float('inf')) -
                fixture['logged_fault']['error_rad']) < .01),
        'realtime_physics_completes': (
            realtime['controller'] == 'COMPLETED' and realtime['fault'] is None),
        'both_slow_physics_cases_reproduce_joint5_tracking_fault': all(
            case['fault'] == 'TRACKING_ERROR' and
            case.get('fault_detail', {}).get('max_error_joint') == 5 for case in slow),
        'all_physics_cases_keep_contacts_zero': all(
            case['max_contacts'] == 0 for case in [realtime] + slow),
        'all_cases_keep_feedback_fresh': all(
            case['max_feedback_age_s'] < config['motion']['feedback_timeout']
            for case in cases.values()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path,
                        default=TEST / 'fixtures/tracking_20261117_005856.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--verify', action='store_true',
                        help='Require the recorded-fault comparison checks to pass')
    args = parser.parse_args()
    config_path = ROOT / 'src/cra/config/robot.yaml'
    config = yaml.safe_load(config_path.read_text())
    fixture = json.loads(args.fixture.read_text())
    if fixture['command']['names'] != [f'joint{i}' for i in range(1, 6)]:
        raise ValueError('Fixture requires canonical joint order')
    preceding = [r for r in fixture['feedback'] if
                 r['stamp_ns'] <= fixture['controller_start_ns']]
    fixture['initial_feedback'] = preceding[-1]
    model = ROOT / 'src/cra' / config['robot']['model']['path']
    args.output.mkdir(parents=True, exist_ok=True)
    result = {'source_bag': fixture['source_bag'],
              'source_database_sha256': fixture['source_database_sha256'],
              'recorded_fault': fixture['logged_fault'],
              'configuration': config['motion'], 'curve': config['trajectory']['curve'],
              'cases': {}, 'limits': [
                  'Isolated deterministic replay, not ROS/viewer end-to-end verification.',
                  'Exact command delivery, steady-clock start and full initial physics '
                  'state were not recorded. Initial actuator target is gravity-balanced.',
                  'Uniform slowdown is a controlled experiment. Recorded-cadence mode '
                  'interpolates five physics steps per feedback interval.',
                  'Human perception/planner callbacks are absent; physical XML collisions '
                  'and production Motion tracking/stall/velocity guards are retained.',
                  'Recorded human capsules are not MuJoCo physical bodies in the live model.']}
    physics_model = mujoco.MjModel.from_xml_path(str(model))
    result['timing_evidence'] = timing_evidence(fixture, physics_model.opt.timestep)
    with tempfile.TemporaryDirectory(prefix='cra_tracking_') as directory:
        library, sources = compile_bridge(directory)
        paths = sources + [args.fixture, config_path, model,
                           ROOT / 'src/simulation/simulation/robot_simulation.py',
                           ROOT / 'src/motion/src/motion_component/motion_component.cpp',
                           Path(__file__).resolve()]
        paths += list((ROOT / 'src/motion/include').rglob('*.hpp'))
        paths += list((model.parent / 'meshes').glob('*.stl'))
        result['source_sha256'] = {str(p.relative_to(ROOT)): sha256(p) for p in paths}
        for name, ratio in [('recorded_feedback', None), ('physics_realtime', 1.),
                            ('physics_half_speed', .5), ('physics_recorded_cadence', None)]:
            summary, rows = run_case(library, config, fixture, model, name, ratio)
            result['cases'][name] = summary
            with (args.output / (name + '.csv')).open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            print(name, json.dumps(summary), flush=True)
    result['comparative_checks'] = comparative_checks(result, fixture, config)
    (args.output / 'diagnosis.json').write_text(json.dumps(result, indent=2) + '\n')
    if args.verify and not all(result['comparative_checks'].values()):
        raise AssertionError(result['comparative_checks'])
    print('comparative_checks', json.dumps(result['comparative_checks']), flush=True)


if __name__ == '__main__':
    main()
