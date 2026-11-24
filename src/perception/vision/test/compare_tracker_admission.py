"""Real-camera tracker enqueue comparison, with an optional MuJoCo trial.

Uses existing camera/workspace/calibration settings and reconstruction YAML.
Blocking and 5 ms use the installed driver. The 0/10 ms experimental
executable is a BUILD_TESTING-only target, never installed.
No images, skeleton positions or capsule geometry are saved; only metadata.
"""

import argparse
import ast
import ctypes
import ctypes.util
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

from tracker_motion_scenario import MotionScenario, evaluate_scenario


ROOT = Path(__file__).resolve().parents[4]
LAUNCH = ROOT / 'src/perception/vision/launch/body_tracking.launch.py'
CONFIG = ROOT / 'src/cra/config/reconstruction.yaml'
ENQUEUE_WAIT_MS = {'blocking': None, 'nonblocking': 0, 'wait-5ms': 5, 'wait-10ms': 10}
GUIDED_PHASES = (
    ('still', 'Hadap kamera dan diam di titik yang sama.'),
    ('arm_motion', 'Ayunkan satu lengan ke samping lalu kembali, sekitar satu kali per detik.'),
    ('partial_occlusion', 'Putar badan sedikit dan sembunyikan lengan itu di balik torso.'),
    ('recovery', 'Hadap kamera kembali, perlihatkan kedua lengan, lalu diam.'),
)


def simulation_config():
    config = Path(get_package_share_directory('cra')) / 'config/robot.yaml'
    if yaml.safe_load(config.read_text())['robot']['mode'] != 'simulation':
        raise SystemExit('The Kinect integration trial requires installed robot.mode: simulation')
    return config


def settings():
    tree = ast.parse(LAUNCH.read_text())
    output = {}
    for assignment in ast.walk(tree):
        if not isinstance(assignment, ast.Assign):
            continue
        names = [t.id for t in assignment.targets if isinstance(t, ast.Name)]
        if any(n in names for n in ('kinect_config', 'workspace_params')):
            output[names[0]] = ast.literal_eval(assignment.value)
        if 'static_tf_publisher' in names:
            value = next(k.value for k in assignment.value.keywords if k.arg == 'arguments')
            output['tf_arguments'] = []
            for element in value.elts:
                if isinstance(element, ast.Call) and element.func.id == 'deg':
                    angle = math.radians(ast.literal_eval(element.args[0]))
                    output['tf_arguments'].append(str(angle))
                else:
                    output['tf_arguments'].append(ast.literal_eval(element))
    assert set(output) == {'kinect_config', 'workspace_params', 'tf_arguments'}
    return output


def binary(package, executable):
    return Path(get_package_prefix(package)) / 'lib' / package / executable


def require_camera(configuration):
    if configuration['kinect_config'].get('recording_file'):
        return
    try:
        sdk = ctypes.CDLL(ctypes.util.find_library('k4a') or 'libk4a.so')
        sdk.k4a_device_get_installed_count.argtypes = []
        sdk.k4a_device_get_installed_count.restype = ctypes.c_uint32
        count = sdk.k4a_device_get_installed_count()
    except OSError as error:
        raise RuntimeError(f'Cannot load Kinect SDK for the camera preflight: {error}') from error
    if count == 0:
        raise RuntimeError(
            'Kinect SDK detected 0 sensors. Check Kinect power, USB connection and '
            'container USB visibility before retrying. No trial was started.')


def require_running(processes, directory):
    for name, process in processes:
        code = process.poll()
        if code is not None:
            log_path = directory / (name + '.log')
            tail = '\n'.join(log_path.read_text(errors='replace').splitlines()[-12:])
            raise RuntimeError(
                f'{name} exited before the trial completed (code {code}). '
                f'Check {log_path}:\n{tail}')


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.)


def wait_for_trial(probe, log_path, mode, timeout, phase_duration, required_processes):
    deadline = time.monotonic() + timeout
    phases_seen = []
    with log_path.open() as log:
        while probe.poll() is None and time.monotonic() < deadline:
            require_running(required_processes, log_path.parent)
            for line in log:
                match = re.fullmatch(r'PROBE_PHASE (\d+)\s*', line)
                if match and phase_duration is not None:
                    index = int(match.group(1))
                    phases_seen.append(index)
                    _, instruction = GUIDED_PHASES[index]
                    print(f'[{mode}] {index + 1}/{len(GUIDED_PHASES)} · '
                          f'{phase_duration:g} detik · {instruction}\a', flush=True)
            time.sleep(.1)
    if probe.poll() is None:
        raise subprocess.TimeoutExpired(probe.args, timeout)
    if (phase_duration is not None and probe.returncode == 0
            and phases_seen != list(range(len(GUIDED_PHASES)))):
        raise RuntimeError(f'Incomplete phase cues for {mode}: {phases_seen}')


def wait_for_motion_scenario(probe, directory, mode, timeout, args, required_processes):
    controller = MotionScenario(args.scenario, args.warmup)
    deadline = time.monotonic() + timeout
    last_received = time.monotonic()
    last_elapsed = 0.

    def apply(actions):
        for kind, value in actions:
            if kind == 'command':
                with (directory / 'control.txt').open('a') as control:
                    control.write(value + '\n')
            else:
                print(f'[{mode}] {value}\a', flush=True)

    with (directory / 'monitor.log').open() as log:
        while probe.poll() is None and time.monotonic() < deadline:
            require_running(required_processes, directory)
            for line in log:
                if line.startswith('PROBE_EVENT '):
                    event = json.loads(line[len('PROBE_EVENT '):])
                    last_elapsed, last_received = event['elapsed_s'], time.monotonic()
                    apply(controller.event(event))
            apply(controller.tick(last_elapsed + time.monotonic() - last_received))
            time.sleep(.05)
    if probe.poll() is None:
        raise subprocess.TimeoutExpired(probe.args, timeout)
    return controller.cues


def run(mode, args, configuration):
    directory = args.output / mode
    directory.mkdir(parents=True, exist_ok=True)
    camera_file = directory / 'camera.params.yaml'
    workspace_file = directory / 'workspace.params.yaml'
    camera_file.write_text(yaml.safe_dump(
        {'/**': {'ros__parameters': configuration['kinect_config']}}))
    workspace_file.write_text(yaml.safe_dump(
        {'/**': {'ros__parameters': configuration['workspace_params']}}))
    wait_ms = ENQUEUE_WAIT_MS[mode]
    installed_driver = wait_ms in (None, 5)
    driver = (binary('azure_kinect_ros_driver', 'node') if installed_driver
              else ROOT / 'build/azure_kinect_ros_driver/tracker_nonblocking_probe')
    driver_args = [driver, '--ros-args', '--params-file', camera_file]
    if wait_ms is not None:
        driver_args.extend(['-p', f'tracker_enqueue_wait_ms:={wait_ms}'])
    monitor = ROOT / 'build/vision/vision_stream_probe'
    processes, logs = [], []
    started = time.monotonic()
    try:
        def launch(name, command):
            log = (directory / (name + '.log')).open('w')
            logs.append(log)
            process = subprocess.Popen([str(x) for x in command], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True,
                                       cwd=directory.resolve())
            processes.append((name, process))
            return process

        launch('tf', [binary('tf2_ros', 'static_transform_publisher'),
                      *configuration['tf_arguments']])
        launch('skeleton', [binary('vision', 'skeleton_transform_node')])
        launch('zone', [binary('vision', 'workspace_monitor_node'), '--ros-args',
                        '--params-file', workspace_file])
        reconstruction_args = [binary('vision', 'reconstruction_node'), '--ros-args',
                               '--params-file', CONFIG]
        if args.scenario:
            # Both scenarios use the same relay topology for a fair comparison.
            reconstruction_args.extend(['-r', '/depth/image_raw:=/tracker_trial/depth'])
        launch('reconstruction', reconstruction_args)
        monitor_args = [monitor, '--warmup', args.warmup, '--duration', args.duration,
                        '--output', directory / 'streams.json']
        if args.mujoco:
            simulation = (
                [binary('simulation', 'robot_simulation')] if args.viewer else
                [sys.executable, Path(__file__).with_name('run_mujoco_tracker_headless.py')]
            )
            launch('simulation', simulation)
            launch('motion', ['ros2', 'launch', 'cra', 'motion.launch.py'])
            launch('planning', [
                'ros2', 'launch', 'cra', 'planning.launch.py',
                f'workspace_path:={ROOT / "src/planning/data/raw_workspace.npz"}',
                f'graph_path:={ROOT / "src/planning/data/workspace_graph.npz"}',
            ])
            for axis, position in zip('xyz', args.target):
                monitor_args.extend([f'--target-{axis}', position])
        if args.scenario:
            control_file = directory / 'control.txt'
            control_file.write_text('')
            monitor_args.extend(['--control-file', control_file])
            print(f'[{mode}] Skenario {args.scenario}: tetap terlihat kamera. '
                  f'Pemanasan {args.warmup:g} detik; instruksi mengikuti status Motion.',
                  flush=True)
        elif args.guided:
            monitor_args.extend(['--phases', len(GUIDED_PHASES)])
            print(f'[{mode}] Bersiap di posisi yang sama. Pemanasan {args.warmup:g} detik.',
                  flush=True)
        required_processes = list(processes)
        probe = launch('monitor', monitor_args)
        launch('driver', driver_args)
        required_processes.append(processes[-1])
        timeout = args.warmup + args.duration + 30.
        scenario_cues = []
        if args.scenario:
            scenario_cues = wait_for_motion_scenario(
                probe, directory, mode, timeout, args, required_processes)
        else:
            wait_for_trial(probe, directory / 'monitor.log', mode, timeout,
                           args.duration / len(GUIDED_PHASES) if args.guided else None,
                           required_processes)
        assert probe.returncode == 0, 'Native monitor failed'
        exits_before_cleanup = {name: p.poll() for name, p in processes}
        data = json.loads((directory / 'streams.json').read_text())
        if args.scenario:
            data['scenario_cues'] = scenario_cues
            data['scenario_result'] = evaluate_scenario(
                data['simulation_events'], args.scenario, args.warmup + args.duration)
            data['scenario_result']['depth_frames_withheld'] = data['depth_frames_withheld']
            data['scenario_result']['input'] = 'Real Kinect; test relay before Reconstruction'
            if args.scenario == 'vision-loss' and data['depth_frames_withheld'] == 0:
                data['scenario_result'].update(status='not_exercised',
                                               reason='No live depth frames were withheld')
        elif args.guided:
            for phase in data['phases']:
                phase['name'], phase['instruction'] = GUIDED_PHASES[phase['index']]
            data['observed_in_still_and_recovery'] = all(
                phase['active_zone_s'] > 0
                and phase['geometry_state_seconds'].get('observed', 0) > 0
                for phase in (data['phases'][0], data['phases'][-1])
            )
            if not data['observed_in_still_and_recovery']:
                print(f'[{mode}] Geometri manusia diperlukan pada fase diam dan pemulihan '
                      'agar hasil uji bisa dibandingkan.', flush=True)
    finally:
        for _, process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
    text = (directory / 'driver.log').read_text(errors='replace')
    logged_waits = re.findall(r'TEST-ONLY tracker admission: enqueue_wait_ms=(\d+)', text)
    if installed_driver:
        admission = re.findall(r'Tracker admission: mode=\w+ enqueue_wait_ms=(-?\d+)', text)
        assert admission == [str(-1 if wait_ms is None else wait_ms)] and not logged_waits, (
            'Installed driver did not report the requested tracker_enqueue_wait_ms; rebuild driver')
    else:
        assert logged_waits == [str(wait_ms)], (
            'Experimental driver did not report the requested enqueue wait')
    reconstruction_log = (directory / 'reconstruction.log').read_text(errors='replace')
    profiles = [{k: float(v) for k, v in re.findall(r'(\w+)=([\d.]+)', line)}
                for line in text.splitlines() if 'Capture profile:' in line]
    skips = [int(v) for v in re.findall(r'sdk_busy_skips=(\d+)', text)]
    data.update(mode=mode, enqueue_wait_ms=wait_ms,
                driver_variant='installed' if installed_driver else 'experimental',
                wall_s=time.monotonic() - started,
                exits_before_cleanup=exits_before_cleanup,
                capture_profiles=profiles, sdk_busy_skips_reported=sum(skips),
                fatal_capture_timeout='No capture after ' in text,
                capture_timeout_warnings=text.count('Capture timeout ('),
                camera_cleanup_completed='Camera stream workers stopped' in text,
                invalid_diagnostics=[line for line in reconstruction_log.splitlines()
                                     if 'Reconstruction invalid:' in line],
                monitor_sha256=hashlib.sha256(monitor.read_bytes()).hexdigest(),
                driver_sha256=hashlib.sha256(driver.read_bytes()).hexdigest())
    (directory / 'summary.json').write_text(json.dumps(data, indent=2) + '\n')
    brief = {k: v for k, v in data.items()
             if k in ('mode', 'active_zone_s', 'geometry_state_seconds', 'streams',
                      'fatal_capture_timeout', 'sdk_busy_skips_reported')}
    print(json.dumps(brief, indent=2), flush=True)
    if args.scenario:
        print(json.dumps(data['scenario_result'], indent=2), flush=True)
    if args.mujoco:
        for event in data['simulation_events']:
            if event['stream'] in ('target', 'command', 'motion'):
                print(f"[{mode}] {event['elapsed_s']:6.2f} s · "
                      f"{event['stream']} {event['state']}", flush=True)
        exited = [name for name in ('simulation', 'planning', 'motion')
                  if exits_before_cleanup[name] is not None]
        if exited:
            raise RuntimeError(f'Simulation trial ended early, check logs in {directory}: {exited}')
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--warmup', type=float, default=15.)
    parser.add_argument('--guided', action='store_true',
                        help='Print four movement cues and collect metrics for each phase')
    parser.add_argument('--mujoco', action='store_true',
                        help='Run the normal simulator, Planning and Motion on the test domain')
    parser.add_argument('--viewer', action='store_true',
                        help='Open the MuJoCo viewer during the optional simulation trial')
    parser.add_argument('--scenario', choices=('blockage', 'vision-loss'),
                        help='Trigger a separate disturbance trial after target EXECUTING')
    parser.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='Target in base_link metres; a scenario sends it after warm-up '
                             'when human geometry and Motion are ready')
    parser.add_argument('--modes', nargs='+', choices=tuple(ENQUEUE_WAIT_MS),
                        default=['blocking', 'nonblocking'])
    args = parser.parse_args()
    args.output = args.output.resolve()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    if os.environ.get('ROS_DOMAIN_ID') != '221' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1')
    assert 1. <= args.duration <= 120. and 0. <= args.warmup <= 60.
    if args.guided and args.duration < 4 * len(GUIDED_PHASES):
        parser.error('--guided needs at least 4 seconds per phase')
    if args.mujoco and (
        not args.guided or args.target is None or not all(map(math.isfinite, args.target))
    ):
        parser.error('--mujoco requires --guided and a finite --target X Y Z')
    if args.target is not None and not args.mujoco:
        parser.error('--target requires --mujoco')
    if args.viewer and not args.mujoco:
        parser.error('--viewer requires --mujoco')
    if args.scenario and (not args.mujoco or args.duration < 20):
        parser.error('--scenario requires --mujoco and --duration of at least 20 seconds')
    config_files = [LAUNCH, CONFIG, ROOT / 'src/cra/config/robot.yaml']
    if args.mujoco:
        config_files.append(simulation_config())
    installed = binary('azure_kinect_ros_driver', 'node').resolve()
    experimental = (ROOT / 'build/azure_kinect_ros_driver/tracker_nonblocking_probe').resolve()
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                if (entry / 'exe').resolve() in (installed, experimental):
                    raise SystemExit(
                        f'Kinect is in use by driver PID {entry.name}. '
                        'Stop its launch from the original terminal, then retry.'
                    )
            except (OSError, RuntimeError):
                pass
    assert len(args.modes) == len(set(args.modes)), 'Use separate output directories for repeats'
    configuration = settings()
    require_camera(configuration)
    args.output.mkdir(parents=True, exist_ok=True)
    original_hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in config_files}
    trials = [run(mode, args, configuration) for mode in args.modes]
    for path, digest in original_hashes.items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest
    scope = 'Kinect and MuJoCo with Planning and Motion' if args.mujoco else 'Kinect Vision only'
    report = dict(scope=scope, domain=221, guided=args.guided, mujoco=args.mujoco,
                  viewer=args.viewer, scenario=args.scenario, configuration=configuration,
                  configuration_sha256=original_hashes, trials=trials,
                  main_launch_nonblocking=False,
                  main_launch_default_enqueue_wait_ms=-1,
                  limitations=['Motion event times are passive DDS receipt times',
                               'People and workspace occupancy can vary between trials',
                               'Phase cues record timing. Participant motion needs confirmation.',
                               'Scenario trials relay depth and are not topology-identical to '
                               'the original FPS comparison',
                               'Scenario verdicts cover command/status evidence, not physical '
                               'braking distance or human pose accuracy',
                               'EMPTY is not proof of active-human geometry continuity',
                               'Observer pair deltas only describe subscriber receipt ordering'])
    (args.output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as error:
        raise SystemExit(str(error)) from None
