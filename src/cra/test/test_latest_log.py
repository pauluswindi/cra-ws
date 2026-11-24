"""Exercise bounded logging without camera, robot, or a running ROS graph."""

import importlib.util
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch


HELPER = Path(__file__).resolve().parents[1] / 'launch/latest_log.py'
spec = importlib.util.spec_from_file_location('cra_latest_log_test', HELPER)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
LatestLog = module.LatestLog


class LatestLogTests(unittest.TestCase):
    def test_new_run_replaces_only_its_own_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            other = path.with_name('keep.log')
            other.write_text('KEEP')
            first = LatestLog(path)
            first.capture('node', 'stdout', b'OLD_RUN\n')
            first.close()
            self.assertIn('OLD_RUN', path.read_text())
            second = LatestLog(path)
            second.capture('node', 'stderr', b'NEW_RUN\n')
            second.close()
            text = path.read_text()
            self.assertNotIn('OLD_RUN', text)
            self.assertIn('[node][stderr] NEW_RUN', text)
            self.assertEqual(other.read_text(), 'KEEP')

    def test_fragmented_unicode_ansi_and_partial_shutdown_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            writer = LatestLog(path)
            message = '\x1b[32mRRT → EXECUTING\x1b[0m\nfinal teardown'.encode()
            for byte in message:
                writer.capture('planning', 'stdout', bytes([byte]))
            writer.close()
            text = path.read_text()
            self.assertIn('RRT → EXECUTING', text)
            self.assertIn('final teardown', text)
            self.assertNotIn('\x1b', text)

    def test_size_limit_keeps_single_bounded_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            writer = LatestLog(path, max_bytes=1024)
            for i in range(30):
                writer.capture('node', 'stdout', (str(i) + ':' + 'x' * 100 + '\n').encode())
            writer.capture('node', 'stdout', b'FINAL_RECORD\n')
            writer.close()
            self.assertLessEqual(path.stat().st_size, 1024)
            self.assertIn('FINAL_RECORD', path.read_text())
            self.assertEqual(list(Path(temporary).iterdir()), [path])

    def test_log_is_flushed_during_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            writer = LatestLog(path)
            try:
                writer.capture('node', 'stdout', b'LIVE_RECORD\n')
                deadline = time.monotonic() + 1.5
                while 'LIVE_RECORD' not in path.read_text() and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertIn('LIVE_RECORD', path.read_text())
                self.assertTrue(writer.thread.is_alive())
            finally:
                writer.close()

    def test_second_writer_cannot_erase_active_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            writer = LatestLog(path)
            try:
                writer.capture('node', 'stdout', b'ACTIVE_RUN\n')
                with self.assertRaises(BlockingIOError):
                    LatestLog(path)
            finally:
                writer.close()
            self.assertIn('ACTIVE_RUN', path.read_text())

    def test_queue_overflow_returns_without_waiting_for_disk(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            entered, release = threading.Event(), threading.Event()
            original = LatestLog._consume

            def slow_consume(writer, *args):
                entered.set()
                release.wait(timeout=2.)
                return original(writer, *args)

            with patch.object(LatestLog, '_consume', slow_consume):
                writer = LatestLog(path, queue_chunks=1)
                try:
                    writer.capture('node', 'stdout', b'FIRST\n')
                    self.assertTrue(entered.wait(timeout=1.))
                    writer.capture('node', 'stdout', b'SECOND\n')
                    writer.capture('node', 'stdout', b'DROPPED\n')
                    self.assertEqual(writer.dropped_chunks, 1)
                finally:
                    release.set()
                    writer.close()
            self.assertIn('dropped_chunks_total=1', path.read_text())

    def test_disabled_and_console_only_modes_do_not_create_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            self.assertEqual(module.logging_actions(dict(enabled=False, latest_file=str(path))), [])
            with patch.dict('os.environ', {'CRA_CONSOLE_ONLY_LOGGING': '1'}):
                self.assertEqual(module.logging_actions(
                    dict(enabled=True, latest_file=str(path))), [])
            self.assertFalse(path.exists())

    def test_disk_failure_disables_capture_without_escaping_worker(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'latest.log'
            with patch.object(LatestLog, '_write', side_effect=OSError('disk unavailable')):
                with patch('sys.stderr'):
                    writer = LatestLog(path)
                    writer.thread.join(timeout=1.)
                    self.assertEqual(writer.error, 'disk unavailable')
                    writer.capture('node', 'stdout', b'not queued\n')
                    self.assertEqual(writer.queue.qsize(), 0)
                    writer.close()

    def test_invalid_settings_are_rejected(self):
        for settings in (dict(enabled='true'), dict(enabled=True, max_size_mb=True),
                         dict(enabled=True, max_size_mb=0), dict(enabled=True, latest_file='')):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                module.logging_actions(settings)


if __name__ == '__main__':
    unittest.main()
