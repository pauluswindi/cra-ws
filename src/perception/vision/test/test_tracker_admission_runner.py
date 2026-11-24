"""Failure-path checks for the tracker comparison harness, without camera access."""

import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import compare_tracker_admission as runner


class TrackerAdmissionRunnerTests(unittest.TestCase):
    def test_missing_sensor_fails_preflight(self):
        sdk = SimpleNamespace(k4a_device_get_installed_count=Mock(return_value=0))
        with patch.object(runner.ctypes, 'CDLL', return_value=sdk):
            with self.assertRaisesRegex(RuntimeError, '0 sensors.*No trial was started'):
                runner.require_camera({'kinect_config': {}})

    def test_detected_sensor_passes_preflight(self):
        sdk = SimpleNamespace(k4a_device_get_installed_count=Mock(return_value=1))
        with patch.object(runner.ctypes, 'CDLL', return_value=sdk):
            runner.require_camera({'kinect_config': {}})

    def test_recording_does_not_require_physical_sensor(self):
        with patch.object(runner.ctypes, 'CDLL') as load:
            runner.require_camera({'kinect_config': {'recording_file': '/tmp/test.mkv'}})
            load.assert_not_called()

    def test_driver_zero_exit_aborts_before_any_phase(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / 'monitor.log').write_text('PROBE_PHASE 0\n')
            (directory / 'driver.log').write_text('Found 0 sensors\nCannot start cameras\n')
            probe = Mock()
            probe.poll.return_value = None
            driver = Mock()
            driver.poll.return_value = 0
            with patch('builtins.print') as announce:
                with self.assertRaisesRegex(RuntimeError, 'driver exited.*code 0') as caught:
                    runner.wait_for_trial(probe, directory / 'monitor.log', 'blocking',
                                          1., 10., [('driver', driver)])
                self.assertIn('Cannot start cameras', str(caught.exception))
                announce.assert_not_called()

    def test_unguided_completion_does_not_require_phase_cues(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / 'monitor.log'
            log_path.write_text('')
            probe = Mock(returncode=0)
            probe.poll.return_value = 0
            runner.wait_for_trial(probe, log_path, 'blocking', 1., None, [])

    def test_guided_trial_still_announces_all_phases(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / 'monitor.log'
            log_path.write_text(''.join(f'PROBE_PHASE {index}\n' for index in range(4)))
            probe = Mock(returncode=0)
            probe.poll.side_effect = [None, 0, 0]
            with patch('builtins.print') as announce, patch.object(runner.time, 'sleep'):
                runner.wait_for_trial(probe, log_path, 'blocking', 1., 10., [])
                self.assertEqual(announce.call_count, 4)


if __name__ == '__main__':
    unittest.main()
