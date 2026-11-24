"""Synthetic input test: real probe, relay and Reconstruction watchdog.

Default Motion/Planning statuses are fixtures. --with-simulation uses real
Planning, C++ Motion and MuJoCo instead. Neither mode uses Kinect hardware.
"""

import argparse
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point, TransformStamped
from msgs.msg import HumanCapsuleArray
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from compare_tracker_admission import ROOT, simulation_config, stop
from tracker_motion_scenario import MotionScenario, evaluate_scenario


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--with-simulation', action='store_true',
                        help='Use real Planning/Motion/MuJoCo instead of mocked statuses')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '221' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Use isolated ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1')
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    if args.with_simulation:
        simulation_config()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('motion_scenario_contract_fixture')
    began = time.monotonic()
    while time.monotonic() - began < 1.:
        rclpy.spin_once(node, timeout_sec=.05)
    if any(node.count_publishers(topic) for topic in (
            '/depth/image_raw', '/vision/human/geometry', '/motion/execution_status',
            '/simulation/set_target')):
        raise SystemExit('Domain 221 is in use; do not run beside another trial')
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'test_depth'
    transform.transform.rotation.w = 1.
    transform.header.stamp = node.get_clock().now().to_msg()
    tf.sendTransform(transform)
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if not args.with_simulation:
        motion = node.create_publisher(String, '/motion/execution_status', durable)
        planning = node.create_publisher(String, '/planning/status', durable)
        commands = node.create_publisher(Bool, '/motion/stop_requested', 10)
        trajectory = node.create_publisher(JointTrajectory, '/motion/joint_path', 10)
    pubs = {name: node.create_publisher(kind, name, 10) for name, kind in (
        ('/depth/image_raw', Image), ('/body_index_map/image_raw', Image),
        ('/depth/camera_info', CameraInfo), ('/vision/skeleton/base_frame', MarkerArray),
        ('/perception/humans_in_zone', MarkerArray))}
    geometry, targets, relayed = [], [], []
    handles = [
        node.create_subscription(HumanCapsuleArray, '/vision/human/geometry',
                                 geometry.append, 10),
        node.create_subscription(Image, '/tracker_trial/depth', relayed.append,
                                 rclpy.qos.qos_profile_sensor_data)]
    if not args.with_simulation:
        handles.append(node.create_subscription(
            Point, '/simulation/set_target', targets.append, 10))
    depth, body, info = Image(), Image(), CameraInfo()
    for message in (depth, body, info):
        message.width, message.height, message.header.frame_id = 128, 96, 'test_depth'
    depth.encoding, depth.step, depth.data = '16UC1', 256, bytes([232, 3]) * (128 * 96)
    body.encoding, body.step, body.data = 'mono8', 128, bytes([1]) * (128 * 96)
    info.k = [128., 0., 64., 0., 96., 48., 0., 0., 1.]
    people = MarkerArray()
    for joint, x, y in ((0, 0., .15), (2, 0., -.1), (26, 0., -.25),
                        (5, -.12, -.1), (6, -.2, 0.), (7, -.25, .1), (9, -.28, .15),
                        (12, .12, -.1), (13, .2, 0.), (14, .25, .1), (16, .28, .15)):
        marker = Marker()
        marker.header.frame_id, marker.id = 'base_link', 100 + joint
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 1.
        people.markers.append(marker)
    marker = Marker()
    marker.header.frame_id, marker.ns, marker.id = 'base_link', 'humans_in_zone', 100
    zone = MarkerArray(markers=[marker])
    control_path = args.output / 'control.txt'
    control_path.write_text('')
    children, logs = [], []
    controller = MotionScenario('vision-loss', 2.)
    events = []
    state, invalid_seen, recovered_seen = 'IDLE', False, False
    last_state_at, stop_at, finish_at = 0., None, None
    duration = 28 if args.with_simulation else 12
    try:
        launch_commands = []
        if args.with_simulation:
            launch_commands.extend([
                ('simulation', [sys.executable, Path(__file__).with_name(
                    'run_mujoco_tracker_headless.py')]),
                ('motion', ['ros2', 'launch', 'cra', 'motion.launch.py']),
                ('planning', ['ros2', 'launch', 'cra', 'planning.launch.py',
                              f'workspace_path:={ROOT / "src/planning/data/raw_workspace.npz"}',
                              f'graph_path:={ROOT / "src/planning/data/workspace_graph.npz"}'])])
        launch_commands.extend([
                ('reconstruction', [Path(get_package_prefix('vision')) / 'lib/vision/'
                                    'reconstruction_node', '--ros-args', '--params-file',
                                    ROOT / 'src/cra/config/reconstruction.yaml', '-r',
                                    '/depth/image_raw:=/tracker_trial/depth']),
                ('probe', [ROOT / 'build/vision/vision_stream_probe', '--warmup', '2',
                           '--duration', str(duration), '--output', args.output / 'streams.json',
                           '--target-x', '.5', '--target-y', '.1', '--target-z', '-.25',
                           '--control-file', control_path])])
        for name, command in launch_commands:
            log = (args.output / (name + '.log')).open('w')
            logs.append(log)
            children.append(subprocess.Popen([str(x) for x in command], stdout=log,
                                             stderr=subprocess.STDOUT, start_new_session=True))
        began = time.monotonic()
        next_frame = 0.
        with (args.output / 'probe.log').open() as log:
            while time.monotonic() - began < duration + 8. and children[-1].poll() is None:
                now = time.monotonic() - began
                assert all(p.poll() is None for p in children[:-1]), 'Required child exited'
                if now >= next_frame:
                    next_frame = now + .05
                    stamp = node.get_clock().now().to_msg()
                    for message in (depth, body, info):
                        message.header.stamp = stamp
                    for marker in people.markers + zone.markers:
                        marker.header.stamp = stamp
                    for topic, message in zip(pubs, (depth, body, info, people, zone)):
                        pubs[topic].publish(message)
                    if not args.with_simulation:
                        motion.publish(String(data=state))
                rclpy.spin_once(node, timeout_sec=.01)
                actions = []
                for line in log:
                    if line.startswith('PROBE_EVENT '):
                        entry = json.loads(line[len('PROBE_EVENT '):])
                        events.append(entry)
                        actions.extend(controller.event(entry))
                actions.extend(controller.tick(now))
                for kind, value in actions:
                    if kind == 'command':
                        with control_path.open('a') as control:
                            control.write(value + '\n')
                if args.with_simulation:
                    continue
                # These statuses are deliberately mocked; only the watchdog is real.
                if targets and state == 'IDLE':
                    state, last_state_at = 'EXECUTING', now
                if state == 'EXECUTING' and geometry and not geometry[-1].valid:
                    invalid_seen = True
                    commands.publish(Bool(data=True))
                    planning.publish(String(data='HUMAN_VISION_UNKNOWN'))
                    state, stop_at = 'STOPPED', now
                if state == 'STOPPED' and geometry[-1].valid:
                    recovered_seen = True
                    commands.publish(Bool(data=False))
                    trajectory.publish(JointTrajectory(points=[JointTrajectoryPoint()]))
                    state, last_state_at = 'EXECUTING', now
                if recovered_seen and state == 'EXECUTING' and now - last_state_at > 1.:
                    state, finish_at = 'COMPLETED', now
                    planning.publish(String(data='COMPLETED'))
            assert children[-1].wait(timeout=3.) == 0
        data = json.loads((args.output / 'streams.json').read_text())
        verdict = evaluate_scenario(data['simulation_events'], 'vision-loss', duration + 2.)
        assert verdict['status'] == 'passed', verdict
        invalid_seen = 'disturbance' in verdict['evidence_s']
        recovered_seen = 'recovery_geometry' in verdict['evidence_s']
        if not args.with_simulation:
            assert invalid_seen and recovered_seen and finish_at is not None
        assert data['depth_frames_withheld'] > 20
        target_count = sum(e['stream'] == 'target' and e['state'] == 'SET'
                           for e in data['simulation_events'])
        assert target_count == 1 and len(relayed) > 0
        scope = ('Real Planning/C++ Motion/MuJoCo and Reconstruction; synthetic camera '
                 'frames and skeletons. No Kinect hardware or live-human verification.'
                 if args.with_simulation else __doc__)
        report = dict(scope=scope, result=verdict, invalid_seen=invalid_seen,
                      recovered_seen=recovered_seen, withheld=data['depth_frames_withheld'],
                      target_count=target_count, relayed_depth_count=len(relayed),
                      mock_stopped_at_s=stop_at)
        (args.output / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        for process in reversed(children):
            stop(process)
        for log in logs:
            log.close()
        del handles
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
