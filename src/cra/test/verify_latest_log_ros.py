"""Verify latest-only capture with actual launch, C++ TF and Python output.

No camera, Planning, Motion, MuJoCo or physical hardware is started. Each
launch uses temporary files and isolated localhost ROS domain 224.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--helper', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    results = []
    with tempfile.TemporaryDirectory(prefix='cra-latest-log-') as temporary:
        root = Path(temporary)
        latest = root / 'logs/latest.log'
        other = root / 'unrelated.log'
        other.write_text('KEEP_THIS_LOG')
        node = root / 'output_probe.py'
        node.write_text('''import os
import sys
import time
marker = sys.argv[1]
print(marker + '_STDOUT', flush=True)
print(marker + '_STDERR', file=sys.stderr, flush=True)
text = (marker + '_RRT → EXECUTING\\n').encode()
os.write(1, text[:len(text)-4])
time.sleep(.05)
os.write(1, text[len(text)-4:])
try:
    while True:
        time.sleep(.05)
except KeyboardInterrupt:
    print(marker + '_TEARDOWN', flush=True)
''')
        harness = root / 'launch_probe.py'
        harness.write_text('''import os
import runpy
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, OpaqueFunction, TimerAction
from launch.events import Shutdown
from launch_ros.actions import Node

helper, latest, node, marker, mode = sys.argv[1:]
if mode == 'console-only':
    os.environ['CRA_CONSOLE_ONLY_LOGGING'] = '1'
settings = dict(enabled=mode != 'disabled', latest_file=latest, max_size_mb=1)
def configure(context):
    return runpy.run_path(helper)['logging_actions'](settings)
description = LaunchDescription([
    OpaqueFunction(function=configure),
    ExecuteProcess(cmd=[sys.executable, node, marker], output='screen'),
    Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
         arguments=['--frame-id', 'latest_log_parent', '--child-frame-id', 'latest_log_child']),
    TimerAction(period=1.5, actions=[EmitEvent(event=Shutdown(reason='log probe complete'))]),
])
service = LaunchService(noninteractive=True)
service.include_launch_description(description)
raise SystemExit(service.run())
''')
        for mode, marker in (('normal', 'FIRST_RUN'), ('normal', 'SECOND_RUN'),
                             ('disabled', 'DISABLED_RUN'), ('console-only', 'CONSOLE_RUN')):
            before = latest.read_bytes() if latest.exists() else None
            environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='224',
                               ROS_LOG_DIR=str(root / (marker + '_ros_logs')))
            environment.pop('CRA_CONSOLE_ONLY_LOGGING', None)
            process = subprocess.Popen(
                [sys.executable, str(harness), str(args.helper.resolve()), str(latest),
                 str(node), marker, mode], env=environment, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
            try:
                stdout, stderr = process.communicate(timeout=15)
            except BaseException:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.communicate(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.communicate()
                raise
            output = stdout + stderr
            checks = dict(clean_exit=process.returncode == 0,
                          console_stdout=marker + '_STDOUT' in output,
                          console_stderr=marker + '_STDERR' in output,
                          console_shutdown=marker + '_TEARDOWN' in output,
                          other_log_preserved=other.read_text() == 'KEEP_THIS_LOG')
            text = latest.read_text() if latest.exists() else ''
            if mode == 'normal':
                checks.update(file_stdout=marker + '_STDOUT' in text,
                              file_stderr=marker + '_STDERR' in text,
                              file_fragmented_unicode=marker + '_RRT → EXECUTING' in text,
                              file_shutdown=marker + '_TEARDOWN' in text,
                              file_cpp_ros_output='latest_log_child' in text,
                              lifecycle_exit='exited code=' in text,
                              clean_writer_close='Log closed; dropped_chunks_total=0' in text,
                              previous_launch_removed=marker != 'SECOND_RUN'
                              or 'FIRST_RUN' not in text,
                              bounded_file=latest.stat().st_size <= 1024 * 1024)
            else:
                checks['disabled_does_not_touch_latest'] = latest.read_bytes() == before
            results.append(dict(mode=mode, marker=marker, checks=checks))
            if not all(checks.values()):
                print(output)
                print(text)
        # Loading the actual CRA description must never reset a log by itself.
        config_share = root / 'share'
        (config_share / 'config').mkdir(parents=True)
        (config_share / 'config/robot.yaml').write_text(
            'logging:\n  enabled: true\n  latest_file: ' + str(latest) + '\n')
        from unittest.mock import patch

        source = Path(__file__).resolve().parents[1] / 'launch/cra.launch.py'
        spec = importlib.util.spec_from_file_location('cra_description_log_test', source)
        module = importlib.util.module_from_spec(spec)
        before = latest.read_bytes()
        with patch('ament_index_python.packages.get_package_share_directory',
                   return_value=str(config_share)):
            spec.loader.exec_module(module)
            module.generate_launch_description()
        assert latest.read_bytes() == before, 'Description inspection truncated latest log'
    report = dict(scope=__doc__, description_load_has_no_file_side_effect=True, trials=results)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    assert all(all(r['checks'].values()) for r in results), 'Latest log verification failed'


if __name__ == '__main__':
    main()
