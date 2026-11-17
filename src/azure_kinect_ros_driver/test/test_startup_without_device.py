"""Exercise native ROS startup failures with all SDK device opens denied."""

import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=Path, required=True)
    parser.add_argument('--shim', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = [
        ('no_device', '0', [], 1, 'no open device or recording'),
        ('device_open_failed', '1', [], 1, 'no open device or recording'),
        ('invalid_retry_budget', '0', ['-p', 'capture_max_consecutive_timeouts:=0'],
         3, 'capture_max_consecutive_timeouts must be positive'),
        ('recording_open_exception', '0', ['-p', 'recording_file:=/nonexistent/cra_test.mkv'],
         3, 'K4A initialization/runtime failure'),
    ]
    rows = []
    for name, count, parameters, code, expected in cases:
        environment = os.environ.copy()
        environment.update(LD_PRELOAD=str(args.shim.resolve()),
                           CRA_TEST_DEVICE_COUNT=count, ROS_DOMAIN_ID='223',
                           ROS_LOCALHOST_ONLY='1')
        result = subprocess.run([str(args.driver.resolve()), '--ros-args'] + parameters,
                                env=environment, capture_output=True, text=True, timeout=15)
        output = result.stdout + result.stderr
        checks = {
            'expected_exit_code': result.returncode == code,
            'expected_reason': expected in output,
            'no_unhandled_exception': 'terminate called' not in output,
            'tracker_not_created_without_calibration':
                'TEST_ONLY_UNEXPECTED_TRACKER_CREATE' not in output,
            'device_open_is_fake': (count != '1' or 'TEST_ONLY_DEVICE_OPEN_DENIED' in output),
        }
        if name in ('no_device', 'device_open_failed'):
            checks['unstarted_workers_cleaned_up'] = 'Camera stream workers stopped' in output
        rows.append({'case': name, 'returncode': result.returncode, 'checks': checks,
                     'output': output})
    report = {'scope': 'Native ROS binary on isolated localhost domain 223; test-only '
                       'SDK shim denies every device open. No camera, frames, robot '
                       'commands or live capture recovery are tested.', 'cases': rows}
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    for row in rows:
        print(row['case'], row['checks'])
    if not all(all(row['checks'].values()) for row in rows):
        raise AssertionError('Native startup regression; see ' + str(args.output))


if __name__ == '__main__':
    main()
