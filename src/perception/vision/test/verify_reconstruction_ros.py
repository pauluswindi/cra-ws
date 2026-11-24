"""Installed reconstruction and watchdog on isolated DDS; synthetic images only.

Uses the shipped reconstruction YAML unchanged. No camera, Planning, Motion
or GUI is started. This verifies transport/freshness, not live Kinect speed.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from msgs.msg import HumanCapsuleArray
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parents[4]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '231':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=231')
    config = ROOT / 'src/cra/config/reconstruction.yaml'
    rclpy.init()
    node = rclpy.create_node('reconstruction_optimization_probe')
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'test_depth'
    transform.transform.rotation.w = 1.
    transform.header.stamp = node.get_clock().now().to_msg()
    tf.sendTransform(transform)
    pubs = {name: node.create_publisher(kind, name, 10) for name, kind in (
        ('/depth/image_raw', Image), ('/body_index_map/image_raw', Image),
        ('/depth/camera_info', CameraInfo), ('/vision/skeleton/base_frame', MarkerArray),
        ('/perception/humans_in_zone', MarkerArray))}
    received = []
    subscription = node.create_subscription(
        HumanCapsuleArray, '/vision/human/geometry', received.append, 10)
    del subscription  # Node retains the subscription.
    width, height = 128, 96
    depth, body, info = Image(), Image(), CameraInfo()
    for message in (depth, body, info):
        message.width, message.height = width, height
        message.header.frame_id = 'test_depth'
    depth.encoding, depth.step = '16UC1', width * 2
    depth.data = bytes([232, 3]) * (width * height)
    body.encoding, body.step = 'mono8', width
    body.data = bytes([1]) * (width * height)
    info.k = [128., 0., 64., 0., 96., 48., 0., 0., 1.]
    people = MarkerArray()
    for joint, x, y in ((0, 0., .15), (2, 0., -.1), (26, 0., -.25),
                        (5, -.12, -.1), (6, -.2, 0.), (7, -.25, .1), (9, -.28, .15),
                        (12, .12, -.1), (13, .2, 0.), (14, .25, .1), (16, .28, .15)):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.id = 100 + joint
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 1.
        people.markers.append(marker)
    zone = MarkerArray()
    marker = Marker()
    marker.header.frame_id, marker.ns, marker.id = 'base_link', 'humans_in_zone', 100
    zone.markers.append(marker)

    def send(empty=False):
        stamp = node.get_clock().now().to_msg()
        for message in (depth, body, info):
            message.header.stamp = stamp
        for marker in people.markers + zone.markers:
            marker.header.stamp = stamp
        pubs['/depth/camera_info'].publish(info)
        pubs['/vision/skeleton/base_frame'].publish(people)
        pubs['/perception/humans_in_zone'].publish(MarkerArray() if empty else zone)
        pubs['/body_index_map/image_raw'].publish(body)
        pubs['/depth/image_raw'].publish(depth)

    def await_result(predicate, send_frames=True, empty=False, timeout=5.):
        until = time.monotonic() + timeout
        next_send = 0.
        while time.monotonic() < until:
            if send_frames and time.monotonic() >= next_send:
                send(empty)
                next_send = time.monotonic() + .05
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError('Timed out waiting for expected geometry transition')

    executable = Path(get_package_prefix('vision')) / 'lib/vision/reconstruction_node'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    process = None
    try:
        with args.output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file', str(config)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            await_result(lambda: any(m.valid and len(m.capsules) == 8 for m in received))
            observed = next(m for m in reversed(received) if m.valid and m.capsules)
            assert observed.header.frame_id == 'base_link'
            assert all(c.radius > 0. for c in observed.capsules)
            received.clear()
            began = time.monotonic()
            await_result(lambda: any(not m.valid for m in received), send_frames=False, timeout=2.)
            invalid_after = time.monotonic() - began
            received.clear()
            await_result(lambda: any(m.valid and not m.capsules for m in received), empty=True)
            report = dict(scope=__doc__, ros_domain=231, capsules=len(observed.capsules),
                          output_frame=observed.header.frame_id,
                          watchdog_invalid_after_s=invalid_after, fresh_empty_verified=True,
                          config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
                          executable=str(executable),
                          executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest())
            args.output.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
