"""Measure native camera streams without Planning/Motion or saving image contents.

Uses every camera setting verbatim from the existing body-tracking launch.
Requires an isolated domain and refuses to start beside another camera driver.
This is a real hardware diagnostic, not a fake-camera unit test.
"""

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import MarkerArray
import yaml


ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / 'src/perception/vision/launch/body_tracking.launch.py'


def camera_settings():
    tree = ast.parse(LAUNCH.read_text())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'kinect_config' for t in node.targets)):
            return ast.literal_eval(node.value)
    raise ValueError('No literal kinect_config in body_tracking.launch.py')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=25.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '229':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=229')
    if not 1. <= args.duration <= 60.:
        raise SystemExit('duration must be between 1 and 60 seconds')
    driver = Path(get_package_prefix('azure_kinect_ros_driver')) / (
        'lib/azure_kinect_ros_driver/node')
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if (entry / 'exe').resolve() == driver.resolve():
                raise SystemExit('Another Kinect driver is running; leave it undisturbed')
        except (OSError, RuntimeError):
            continue
    settings = camera_settings()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    config_path = args.output.with_suffix('.params.yaml')
    log_path = args.output.with_suffix('.log')
    config_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
    rclpy.init()
    node = rclpy.create_node('capture_stream_probe')
    rows = {}
    subscriptions = []
    for topic, kind in (
        ('/depth/image_raw', Image), ('/rgb/image_raw/compressed', CompressedImage),
        ('/body_index_map/image_raw', Image), ('/body_tracking_data', MarkerArray),
    ):
        rows[topic] = []

        def callback(message, name=topic):
            now = time.monotonic()
            header = (message.markers[0].header if isinstance(message, MarkerArray)
                      and message.markers else getattr(message, 'header', None))
            age = None
            if header is not None:
                stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
                age = (node.get_clock().now().nanoseconds - stamp) / 1e6
            if len(rows[name]) < 10000:
                rows[name].append((now, age))

        subscriptions.append(node.create_subscription(
            kind, topic, callback, qos_profile_sensor_data))
    began = time.monotonic()
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                [str(driver), '--ros-args', '--params-file', str(config_path.resolve())],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while time.monotonic() - began < args.duration and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
            natural_exit = process.poll()
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
    streams = {}
    for topic, samples in rows.items():
        stamps = [s[0] for s in samples]
        ages = [s[1] for s in samples if s[1] is not None]
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        streams[topic] = dict(
            count=len(samples),
            active_rate_hz=(len(stamps) - 1) / (stamps[-1] - stamps[0]) if gaps else None,
            max_callback_gap_s=max(gaps, default=None),
            median_capture_age_ms=statistics.median(ages) if ages else None,
            max_capture_age_ms=max(ages, default=None))
    log_text = log_path.read_text(errors='replace')
    report = dict(
        scope='Native hardware and driver, synthetic-free input; isolated domain 229. '
              'No Planning/Motion/reconstruction/viewer; image payloads are not saved. '
              'Stream subscriptions exercise capture, tracker and publication, '
              'but do not reproduce the complete CRA workload.',
        requested_duration_s=args.duration, wall_s=time.monotonic() - began,
        settings=settings, natural_exit_code=natural_exit,
        streams=streams, capture_timeout_messages=log_text.count('Capture timeout ('),
        fatal_capture_timeout='No capture after ' in log_text,
        sdk_color_queue_drops=log_text.count('type:Color'),
        cleanup_completed='Camera stream workers stopped' in log_text,
        camera_launch_sha256=hashlib.sha256(LAUNCH.read_bytes()).hexdigest(),
        diagnostic_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        driver_log=str(log_path), parameters_file=str(config_path))
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
