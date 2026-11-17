"""Compare real C++/Python ROS log output in normal and demo launch modes.

No CRA pipeline, camera or robot is started. Run in the ROS container after
building cra: python3 src/cra/test/test_demo_console.py --demo <installed demo>.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', type=Path, required=True)
    options = parser.parse_args()
    results = []
    with tempfile.TemporaryDirectory(prefix='cra_demo_test_') as temporary:
        root = Path(temporary)
        node = root / 'probe.py'
        node.write_text('''import rclpy
rclpy.init()
node = rclpy.create_node('cra_demo_python_probe')
node.get_logger().info('DEMO_PYTHON_INFO')
node.get_logger().warning('DEMO_PYTHON_WARNING')
node.destroy_node()
rclpy.shutdown()
''')
        harness = root / 'launch_probe.py'
        harness.write_text('''import os
import runpy
import sys

demo = runpy.run_path(sys.argv[1])
is_demo = sys.argv[2] == 'demo'
if is_demo:
    demo['configure_console_logging']()
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from launch_ros.actions import Node

actions = demo['console_actions']() if is_demo else []
actions += [
    Node(package='tf2_ros', executable='static_transform_publisher',
         name='cra_demo_cpp_probe', output='screen', arguments=[
             '--frame-id', 'cra_demo_parent', '--child-frame-id', 'cra_demo_child']),
    Node(executable=sys.executable, arguments=[sys.argv[3]], output='screen'),
    TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='probe complete'))]),
]
service = LaunchService(noninteractive=True)
service.include_launch_description(LaunchDescription(actions))
raise SystemExit(service.run())
''')
        for mode in ('normal', 'demo'):
            mode_root = root / mode
            mode_root.mkdir()
            environment = dict(os.environ, ROS_LOG_DIR=str(mode_root / 'logs'),
                               ROS_HOME=str(mode_root / 'ros_home'),
                               ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='217')
            environment.pop('OVERRIDE_LAUNCH_PROCESS_OUTPUT', None)
            result = subprocess.run(
                [sys.executable, str(harness), str(options.demo.resolve()), mode, str(node)],
                env=environment, text=True, capture_output=True, timeout=20,
            )
            output = result.stdout + result.stderr
            log_files = [str(p.relative_to(mode_root)) for p in mode_root.rglob('*.log')]
            checks = {
                'clean_exit': result.returncode == 0,
                'python_info_on_console': 'DEMO_PYTHON_INFO' in output,
                'python_warning_on_console': 'DEMO_PYTHON_WARNING' in output,
                'cpp_transform_on_console': 'cra_demo_child' in output,
                'expected_file_logging': bool(log_files) == (mode == 'normal'),
                'demo_did_not_create_log_directory':
                    mode != 'demo' or not (mode_root / 'logs').exists(),
            }
            results.append({'mode': mode, 'checks': checks, 'log_files': log_files})
            if not all(checks.values()):
                print(output)
        print(json.dumps(results, indent=2))
        if not all(all(row['checks'].values()) for row in results):
            raise AssertionError('Console-only demo regression')


if __name__ == '__main__':
    main()
