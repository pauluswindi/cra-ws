"""Native preview callbacks and counters on domain 232; synthetic JPEG only.

A temporary, test-only interposer suppresses imshow/waitKey, keeping decode,
resize, overlay and normal timers. No camera, Planning or Motion is started.
This cannot establish interactive GUI FPS under the complete CRA workload.
"""

import argparse
from array import array
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CompressedImage, Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--executable', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '232':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=232')
    rclpy.init()
    node = rclpy.create_node('preview_stream_probe')
    publisher = node.create_publisher(CompressedImage, '/rgb/image_raw/compressed', 1)
    raw_publisher = node.create_publisher(Image, '/rgb/image_raw', 1)
    image = np.random.default_rng(42).integers(0, 256, (1536, 2048, 3), dtype=np.uint8)
    image = cv2.GaussianBlur(image, (5, 5), 0.)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok
    message = CompressedImage()
    message.header.frame_id = 'rgb_camera'
    message.format = 'jpeg'
    message.data = array('B', encoded.tobytes())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='cra_preview_') as directory:
            directory = Path(directory)
            source, library = directory / 'headless.cpp', directory / 'headless.so'
            source.write_text('#include <opencv2/highgui.hpp>\n'
                              'namespace cv { void imshow(const String&, InputArray) {}\n'
                              'int waitKey(int) { return -1; } }\n')
            includes = shlex.split(subprocess.check_output(
                ['pkg-config', '--cflags', 'opencv4'], text=True))
            subprocess.run(['c++', '-shared', '-fPIC', *includes, str(source),
                            '-o', str(library)], check=True)
            environment = os.environ.copy()
            environment['LD_PRELOAD'] = str(library)
            executable = args.executable or (
                Path(get_package_prefix('vision')) / 'lib/vision/visual_node')
            log_path = args.output.with_suffix('.log')
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    str(executable), '--ros-args', '-p', 'display_target_fps:=20.0',
                    '-p', 'display_scale:=0.5'], env=environment,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                began = time.monotonic()
                next_send = began
                sent = 0
                while time.monotonic() - began < 12.:
                    assert process.poll() is None, 'Native preview process exited'
                    if time.monotonic() >= next_send:
                        message.header.stamp = node.get_clock().now().to_msg()
                        publisher.publish(message)
                        sent += 1
                        next_send = max(next_send + 1. / 30., time.monotonic())
                    rclpy.spin_once(node, timeout_sec=.002)
                raw = Image()
                raw.header.frame_id = 'rgb_camera'
                raw.width, raw.height, raw.step, raw.encoding = 64, 48, 192, 'bgr8'
                raw.data = bytes(64 * 48 * 3)
                for raw_phase in (True, False):
                    until = time.monotonic() + 2.
                    next_send = 0.
                    while time.monotonic() < until:
                        if time.monotonic() >= next_send:
                            stamp = node.get_clock().now().to_msg()
                            if raw_phase:
                                raw.header.stamp = stamp
                                raw_publisher.publish(raw)
                            message.header.stamp = stamp
                            publisher.publish(message)
                            next_send = time.monotonic() + .05
                        rclpy.spin_once(node, timeout_sec=.002)
                # Stop publication to exercise CAMERA SIGNAL LOST without a camera.
                until = time.monotonic() + 1.5
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.01)
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5.)
            logs = log_path.read_text()
            profiles = []
            for line in logs.splitlines():
                if 'Preview profile:' in line:
                    profiles.append({k: float(v) for k, v in re.findall(
                        r'(\w+)=([\d.]+)', line)})
            assert len(profiles) >= 2, logs[-2000:]
            assert all(p['decoded_fps'] > 0 and p['displayed_fps'] > 0 for p in profiles)
            assert 'Color source switched back to /rgb/image_raw' in logs
            assert logs.count('Color source switched to /rgb/image_raw/compressed') >= 2
            report = dict(scope=__doc__, source_fps=30, preview_target_fps=20,
                          scale=.5, source_width=2048, source_height=1536,
                          frames_sent=sent, preview_profiles=profiles,
                          camera_or_robot_started=False, gui_interposed=True,
                          raw_priority_and_compressed_fallback_verified=True,
                          process_survived_input_loss=True)
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
