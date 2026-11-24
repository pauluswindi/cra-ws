"""Bounded asynchronous stdout/stderr capture for the normal CRA launch."""

import atexit
from datetime import datetime
import fcntl
import os
from pathlib import Path
import queue
import re
import sys
import threading
import time


class LatestLog:
    """Queue on the launch thread; format and write only on the worker."""

    CHUNK_BYTES = 8192
    PARTIAL_LINE_BYTES = 65536

    def __init__(self, path, max_bytes=20 * 1024 * 1024, queue_chunks=256):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            # Lock before truncation: a second launch must not erase an active log.
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.ftruncate(descriptor, 0)
            self.file = os.fdopen(descriptor, 'wb', buffering=65536)
        except BaseException:
            os.close(descriptor)
            raise
        self.max_bytes = max_bytes
        self.queue = queue.Queue(maxsize=queue_chunks)
        self.finished = threading.Event()
        self.dropped_chunks = 0
        self.error = None
        self.size = 0
        self.pending = {}
        self.ansi = re.compile(rb'\x1b\[[0-?]*[ -/]*[@-~]')
        self.thread = threading.Thread(target=self._run, name='cra-latest-log', daemon=True)
        self.thread.start()

    def capture(self, name, stream, payload):
        if self.finished.is_set() or self.error is not None:
            return
        for start in range(0, len(payload), self.CHUNK_BYTES):
            try:
                self.queue.put_nowait((name, stream, payload[start:start + self.CHUNK_BYTES]))
            except queue.Full:
                self.dropped_chunks += 1

    def _write(self, payload):
        if self.size + len(payload) > self.max_bytes:
            self.file.seek(0)
            self.file.truncate()
            notice = b'# Size limit reached; earlier part of this launch was replaced.\n'
            self.file.write(notice)
            self.size = len(notice)
        remaining = max(0, self.max_bytes - self.size)
        payload = payload[-remaining:] if remaining else b''
        self.file.write(payload)
        self.size += len(payload)

    def _line(self, name, stream, line):
        prefix = f'[{name}][{stream}] '.encode('utf-8', errors='replace')
        self._write(prefix + self.ansi.sub(b'', line).rstrip(b'\r') + b'\n')

    def _consume(self, name, stream, payload):
        key = (name, stream)
        data = self.pending.pop(key, b'') + payload
        lines = data.split(b'\n')
        for line in lines[:-1]:
            self._line(name, stream, line)
        tail = lines[-1]
        if len(tail) >= self.PARTIAL_LINE_BYTES:
            self._line(name, stream, tail)
        elif tail:
            self.pending[key] = tail

    def _run(self):
        try:
            self._write(f'# CRA latest log started {datetime.now().astimezone().isoformat()}\n'
                        '# Captured child stdout/stderr; no image or rosbag recording.\n'.encode())
            last_flush, reported_drops = time.monotonic(), 0
            while not self.finished.is_set() or not self.queue.empty():
                try:
                    self._consume(*self.queue.get(timeout=.1))
                except queue.Empty:
                    pass
                if time.monotonic() - last_flush >= .5:
                    if self.dropped_chunks != reported_drops:
                        self._write(f'# Queue overflow: dropped_chunks_total='
                                    f'{self.dropped_chunks}\n'.encode())
                        reported_drops = self.dropped_chunks
                    self.file.flush()
                    last_flush = time.monotonic()
            for (name, stream), tail in self.pending.items():
                self._line(name, stream, tail)
            self.pending.clear()
            self._write(f'# Log closed; dropped_chunks_total={self.dropped_chunks}\n'.encode())
        except OSError as error:
            self.error = str(error)
            print(f'CRA latest log disabled after write failure: {error}', file=sys.stderr)
        finally:
            try:
                self.file.close()
            except OSError as error:
                if self.error is None:
                    self.error = str(error)
                    print(f'CRA latest log close failed: {error}', file=sys.stderr)

    def close(self):
        self.finished.set()
        # This runs after LaunchService exits, not from a node's control loop.
        self.thread.join(timeout=2.)


def logging_actions(settings):
    """Called by OpaqueFunction at execution, never during --show-args."""
    if os.environ.get('CRA_CONSOLE_ONLY_LOGGING') == '1':
        return []
    enabled = settings.get('enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('logging.enabled must be a YAML boolean')
    if not enabled:
        return []
    path = settings.get('latest_file', '/tmp/cra_latest.log')
    limit = settings.get('max_size_mb', 20)
    if not isinstance(path, str) or not path.strip():
        raise ValueError('logging.latest_file must be a nonempty path string')
    if type(limit) is not int or not 1 <= limit <= 1024:
        raise ValueError('logging.max_size_mb must be an integer from 1 to 1024')

    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
    import launch.logging

    logger = launch.logging.get_logger('cra.latest_log')
    try:
        writer = LatestLog(path, max_bytes=limit * 1024 * 1024)
    except OSError as error:
        logger.warning(f'Latest log unavailable; console output continues: {error}')
        return []
    # Closing on Shutdown would miss child teardown output. CLI atexit runs
    # after LaunchService has finished collecting all process output instead.
    atexit.register(writer.close)
    logger.info(f'Latest CRA log: {writer.path} (replaced at startup, limit {limit} MiB)')

    def stdout(event):
        writer.capture(event.process_name, 'stdout', event.text)

    def stderr(event):
        writer.capture(event.process_name, 'stderr', event.text)

    def started(event, _context):
        writer.capture('launch', 'event',
                       f'{event.process_name} started pid={event.pid}\n'.encode())

    def exited(event, _context):
        writer.capture('launch', 'event',
                       f'{event.process_name} exited code={event.returncode}\n'.encode())

    return [
        RegisterEventHandler(OnProcessIO(on_stdout=stdout, on_stderr=stderr)),
        RegisterEventHandler(OnProcessStart(on_start=started)),
        RegisterEventHandler(OnProcessExit(on_exit=exited)),
    ]
