"""Bounded, passive timing observations; never authorize motion or refresh data."""

from collections import deque
from functools import wraps
import time


def profiled(stage, new_cycle=False):
    """Observe a manager method, including early returns and exceptions."""
    def decorate(function):
        @wraps(function)
        def wrapped(self, *args, **kwargs):
            profiler = self.profiler
            if not profiler.enabled:
                return function(self, *args, **kwargs)
            if new_cycle:
                profiler.start_cycle()
            return profiler.call(stage, function, self, *args, **kwargs)
        return wrapped
    return decorate


class PlanningProfiler:
    """Stage times are inclusive. Budgets use the checker's existing deadlines."""

    def __init__(self, checker, enabled=False):
        self.checker = checker
        self.enabled = bool(enabled)
        self.cycle = 0
        self.records = deque(maxlen=64)
        self.responses = deque(maxlen=16)
        self.command = None

    def start_cycle(self):
        self.cycle += 1
        self.records.clear()
        self.records.extend(self.responses)
        self.responses.clear()

    def budget_ms(self):
        """Read remaining capture/receipt lifetime without touching the checker."""
        if getattr(self.checker, 'human_status', None) != 'OBSERVED':
            return None
        deadlines = []
        received = getattr(self.checker, 'human_received_at', None)
        if received is not None:
            deadlines.append(received + self.checker.human_data_timeout)
        expires = getattr(self.checker, 'human_expires_at', None)
        if expires is not None:
            deadlines.append(expires)
        if not deadlines:
            return None
        return round((min(deadlines) - time.monotonic()) * 1000., 3)

    def call(self, stage, function, *args, **kwargs):
        if not self.enabled:
            return function(*args, **kwargs)
        before = self.budget_ms()
        revision = getattr(self.checker, 'environment_revision', None)
        began = time.perf_counter()
        outcome = 'ERROR'
        try:
            result = function(*args, **kwargs)
            outcome = ('NONE' if result is None else
                       'FALSE' if result is False else
                       'TRUE' if result is True else 'RETURNED')
            return result
        finally:
            self.records.append({
                'stage': stage,
                'wall_ms': round((time.perf_counter() - began) * 1000., 3),
                'remaining_before_ms': before,
                'remaining_after_ms': self.budget_ms(),
                'revision_before': revision,
                'revision_after': getattr(self.checker, 'environment_revision', None),
                'outcome': outcome,
            })

    def command_sent(self, action, sent_at):
        if self.enabled:
            self.command = (action, sent_at, self.budget_ms(), self.cycle,
                            getattr(self.checker, 'environment_revision', None))

    def command_response(self, pending_action, status):
        """Measure observed acknowledgement latency, not transport latency alone."""
        if not self.enabled or self.command is None:
            return
        action, began, before, cycle, revision = self.command
        expected = {'STOP': 'STOPPED', 'RELEASE': 'IDLE', 'EXECUTE': 'EXECUTING'}
        if pending_action == action and status == expected.get(action):
            self.responses.append({
                'stage': action.lower() + '_ack_wait',
                'origin_cycle': cycle,
                'wall_ms': round((time.monotonic() - began) * 1000., 3),
                'remaining_before_ms': before,
                'remaining_after_ms': self.budget_ms(),
                'revision_before': revision,
                'revision_after': getattr(self.checker, 'environment_revision', None),
                'outcome': status,
            })
            self.command = None
        elif status in ('STOPPED', 'CONTROL_ERROR', 'INVALID_FEEDBACK', 'FEEDBACK_TIMEOUT'):
            self.command = None
