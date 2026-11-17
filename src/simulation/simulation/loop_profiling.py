"""Bounded, passive measurements of simulation wall time and physics time."""

import math
import time


STAGES = (
    'ros_callbacks', 'viewer_lock_wait', 'control', 'physics',
    'feedback_and_markers', 'viewer_sync', 'sleep', 'profile_output',
)


class SimulationLoopProfiler:
    """Keep count/sum/max per stage; never adjust stepping, sleep or commands."""

    def __init__(self, timestep, enabled=True, interval=5.0, clock=None):
        if not isinstance(enabled, bool):
            raise ValueError('Simulation profiling enabled must be boolean')
        if not all(math.isfinite(v) and v > 0 for v in (timestep, interval)):
            raise ValueError('Profiling timestep and interval must be positive and finite')
        self.enabled = enabled
        self.timestep = float(timestep)
        self.interval = float(interval)
        self.clock = clock or time.perf_counter
        self.window_index = 0
        self.started = False

    def reset(self, physics_time):
        """Start an observation window after viewer startup, before stepping."""
        if not self.enabled:
            return
        self._reset_window(self.clock(), physics_time)
        self.started = True

    def _reset_window(self, now, physics_time):
        self.window_start = now
        self.physics_start = physics_time
        self.loops = 0
        self.over_budget = 0
        self.requested_sleep = 0.0
        self.loop_total = 0.0
        self.loop_max = 0.0
        self.work_total = 0.0
        self.work_max = 0.0
        self.stages = {name: [0, 0.0, 0.0] for name in STAGES}

    def begin_loop(self):
        if self.enabled:
            self.loop_start = self.clock()

    def start_stage(self):
        return self.clock() if self.enabled else None

    def finish_stage(self, name, began):
        if began is None:
            return
        duration = self.clock() - began
        values = self.stages[name]
        values[0] += 1
        values[1] += duration
        values[2] = max(values[2], duration)

    def finish_loop(self, physics_time, work_elapsed, requested_sleep):
        """Return at most one summary per interval, after the existing sleep."""
        if not self.enabled:
            return None
        if not self.started:
            raise RuntimeError('Reset simulation profiling before starting the loop')
        now = self.clock()
        duration = now - self.loop_start
        self.loops += 1
        self.loop_total += duration
        self.loop_max = max(self.loop_max, duration)
        self.work_total += work_elapsed
        self.work_max = max(self.work_max, work_elapsed)
        self.over_budget += int(work_elapsed > self.timestep)
        self.requested_sleep += requested_sleep
        wall = now - self.window_start
        if wall < self.interval:
            return None
        physics_delta = physics_time - self.physics_start
        stage_total = sum(value[1] for value in self.stages.values())
        self.window_index += 1
        report = {
            'schema_version': 1,
            'window_index': self.window_index,
            'wall_elapsed_s': wall,
            'physics_start_s': self.physics_start,
            'physics_end_s': physics_time,
            'physics_advanced_s': physics_delta,
            'real_time_factor': physics_delta / wall if physics_delta >= 0 else None,
            'physics_time_reset': physics_delta < 0,
            'physics_step_s': self.timestep,
            'loops': self.loops,
            'step_hz': self.loops / wall,
            'feedback_hz': self.stages['feedback_and_markers'][0] / wall,
            'work_over_budget_loops': self.over_budget,
            'work_mean_ms': self.work_total * 1000 / self.loops,
            'work_max_ms': self.work_max * 1000,
            'loop_mean_ms': self.loop_total * 1000 / self.loops,
            'loop_max_ms': self.loop_max * 1000,
            'requested_sleep_total_ms': self.requested_sleep * 1000,
            'unattributed_wall_ms': (wall - stage_total) * 1000,
            'stages': {
                name: {
                    'count': count,
                    'total_ms': total * 1000,
                    'mean_ms': total * 1000 / count if count else 0.0,
                    'max_ms': maximum * 1000,
                    'wall_percent': total * 100 / wall,
                }
                for name, (count, total, maximum) in self.stages.items()
            },
        }
        self._reset_window(now, physics_time)
        return report
