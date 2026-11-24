"""Headless replay transport check on domain 226; no Motion node or camera."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '226':
        parser.error('Use isolated ROS_DOMAIN_ID=226')
    rclpy.init()
    node = Node('rrt_replay_probe')
    commands, texts, steps = [], [], set()
    counts = {layer: 0 for layer in ('start_tree', 'goal_tree', 'final_path')}

    def markers(layer, message):
        counts[layer] = max(counts[layer], sum(len(m.points) for m in message.markers))

    def status(message):
        for marker in message.markers:
            if marker.text:
                texts.append(marker.text)
                for event in ('SAMPLE_GENERATED', 'NEAREST_SELECTED', 'EXTEND_ACCEPTED',
                              'EXTEND_REJECTED', 'CONNECT_STARTED', 'CONNECT_ADVANCED',
                              'CONNECT_REACHED', 'PATH_FOUND'):
                    if event in marker.text:
                        steps.add(event)

    handles = [node.create_subscription(
        MarkerArray, '/planning/rrt/' + layer,
        lambda m, name=layer: markers(name, m), 10) for layer in counts]
    handles += [
        node.create_subscription(MarkerArray, '/planning/rrt/search_step', status, 10),
        node.create_subscription(JointTrajectory, '/motion/joint_path',
                                 lambda _: commands.append('EXECUTE'), 10),
        node.create_subscription(Bool, '/motion/stop_requested',
                                 lambda _: commands.append('STOP_OR_RELEASE'), 10),
    ]
    child = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            child = subprocess.Popen([sys.executable, '-m', 'planning.rrt_replay',
                                      '--file', str(args.file), '--rate', '20'],
                                     stdout=log, stderr=log)
            began = time.monotonic()
            while time.monotonic() - began < 45.:
                rclpy.spin_once(node, timeout_sec=.02)
                if counts['final_path'] and 'PATH_FOUND' in steps:
                    break
                if child.poll() is not None:
                    log.seek(0)
                    raise AssertionError(log.read())
            assert all(counts.values()), counts
            assert {'SAMPLE_GENERATED', 'NEAREST_SELECTED', 'EXTEND_REJECTED',
                    'CONNECT_STARTED', 'PATH_FOUND'}.issubset(steps), steps
            assert any('OFFLINE HISTORICAL REPLAY' in t for t in texts), texts
            assert not commands, commands
            report = dict(scope=__doc__, ros_domain=226, marker_points_received=counts,
                          steps_seen=sorted(steps), historical_label_seen=True,
                          motion_commands=commands, elapsed_s=time.monotonic() - began)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
            assert handles
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
