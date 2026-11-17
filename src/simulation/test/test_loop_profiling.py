"""Check timing attribution and preserve the production simulation loop order."""

import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from simulation.loop_profiling import SimulationLoopProfiler, STAGES
import test_hold_control as hold


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class LoopProfilingTests(unittest.TestCase):
    def test_slow_physics_ratio_and_exclusive_stage_totals(self):
        clock = Clock()
        profiler = SimulationLoopProfiler(.002, interval=.02, clock=clock)
        profiler.reset(physics_time=0.)
        report = None
        for index in range(10):
            profiler.begin_loop()
            began = profiler.start_stage()
            clock.advance(.0025)
            profiler.finish_stage('ros_callbacks', began)
            began = profiler.start_stage()
            clock.advance(.0015)
            profiler.finish_stage('physics', began)
            report = profiler.finish_loop((index + 1) * .002, .004, 0.)
            if report:
                break
        self.assertIsNotNone(report)
        self.assertAlmostEqual(report['real_time_factor'], .5)
        self.assertEqual(report['work_over_budget_loops'], report['loops'])
        self.assertAlmostEqual(report['step_hz'], 250.)
        self.assertAlmostEqual(report['stages']['ros_callbacks']['mean_ms'], 2.5)
        self.assertAlmostEqual(report['stages']['physics']['mean_ms'], 1.5)
        self.assertAlmostEqual(sum(s['total_ms'] for s in report['stages'].values()) +
                               report['unattributed_wall_ms'],
                               report['wall_elapsed_s'] * 1000.)
        json.dumps(report, allow_nan=False)

    def test_sleep_overshoot_and_unmeasured_time_remain_visible(self):
        clock = Clock()
        profiler = SimulationLoopProfiler(.002, interval=.004, clock=clock)
        profiler.reset(0.)
        profiler.begin_loop()
        began = profiler.start_stage()
        clock.advance(.004)
        profiler.finish_stage('sleep', began)
        clock.advance(.002)
        report = profiler.finish_loop(.002, .001, .001)
        self.assertAlmostEqual(report['stages']['sleep']['total_ms'], 4.)
        self.assertAlmostEqual(report['requested_sleep_total_ms'], 1.)
        self.assertAlmostEqual(report['unattributed_wall_ms'], 2.)
        self.assertEqual(report['work_over_budget_loops'], 0)
        self.assertAlmostEqual(report['real_time_factor'], 1 / 3)

    def test_windows_reset_and_previous_output_cost_is_accounted(self):
        clock = Clock()
        profiler = SimulationLoopProfiler(.002, interval=.004, clock=clock)
        profiler.reset(0.)
        profiler.begin_loop()
        clock.advance(.005)
        first = profiler.finish_loop(.002, .005, 0.)
        began = profiler.start_stage()
        clock.advance(.001)
        profiler.finish_stage('profile_output', began)
        profiler.begin_loop()
        clock.advance(.004)
        second = profiler.finish_loop(.004, .004, 0.)
        self.assertEqual((first['window_index'], second['window_index']), (1, 2))
        self.assertEqual(second['loops'], 1)
        self.assertAlmostEqual(second['physics_advanced_s'], .002)
        self.assertAlmostEqual(second['wall_elapsed_s'], .005)
        self.assertAlmostEqual(second['stages']['profile_output']['total_ms'], 1.)
        self.assertEqual(set(profiler.stages), set(STAGES))
        self.assertTrue(all(v == [0, 0., 0.] for v in profiler.stages.values()))

    def test_backward_physics_time_has_no_misleading_factor(self):
        clock = Clock()
        profiler = SimulationLoopProfiler(.002, interval=.004, clock=clock)
        profiler.reset(10.)
        profiler.begin_loop()
        clock.advance(.005)
        report = profiler.finish_loop(.002, .005, 0.)
        self.assertTrue(report['physics_time_reset'])
        self.assertIsNone(report['real_time_factor'])

    def test_disabled_profiler_does_not_read_the_clock_or_collect_samples(self):
        profiler = SimulationLoopProfiler(.002, enabled=False,
                                          clock=lambda: self.fail('Clock was read'))
        profiler.reset(0.)
        profiler.begin_loop()
        began = profiler.start_stage()
        profiler.finish_stage('physics', began)
        self.assertIsNone(profiler.finish_loop(.002, .001, .001))
        self.assertFalse(hasattr(profiler, 'stages'))

    def test_invalid_configuration(self):
        for arguments in [{'timestep': 0.}, {'timestep': float('nan')},
                          {'timestep': .002, 'interval': float('inf')},
                          {'timestep': .002, 'enabled': 'true'}]:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                SimulationLoopProfiler(**arguments)


class SimulationLoopIntegrationTests(unittest.TestCase):
    def run_loop(self, enabled, sync_delay=.006):
        """Run real physics/control with a fake viewer, ROS spin and wall clock."""
        fixture = hold.SimulationHoldTests()
        fixture.setUp()
        try:
            robot = fixture.robot
            clock = Clock()
            robot.loop_profiler = SimulationLoopProfiler(
                robot.dt, enabled=enabled, interval=.02, clock=clock)
            robot.velocity_callback(SimpleNamespace(data=[.1, -.05, .08, 0., .1]))
            trace, messages, sleep_requests = [], [], []
            real_update = robot.update_control
            real_step = mujoco.mj_step
            module = robot.run.__globals__

            def update():
                trace.append('control')
                real_update()
                clock.advance(.0001)

            def physics(model, data):
                trace.append('physics')
                real_step(model, data)
                clock.advance(.0001)

            def feedback():
                trace.append('feedback')
                clock.advance(.0004)

            def spin(*args, **kwargs):
                trace.append('spin')
                self.assertEqual(kwargs, {'timeout_sec': 0.0})
                clock.advance(.0002)

            def sleep(seconds):
                trace.append('sleep')
                sleep_requests.append(seconds)
                clock.advance(seconds + .0002)

            def log(message):
                messages.append(message)
                if message.startswith('Simulation profile: '):
                    clock.advance(.0001)

            class Viewer:
                def __init__(self):
                    self.steps = 0

                def __enter__(self):
                    return self

                def __exit__(self, *args):
                    pass

                def is_running(self):
                    return self.steps < 20

                def lock(self):
                    trace.append('lock')
                    viewer = self

                    class Lock:
                        def __enter__(self):
                            clock.advance(.00005)

                        def __exit__(self, *args):
                            viewer.steps += 1
                            trace.append('unlock')

                    return Lock()

                def sync(self):
                    trace.append('sync')
                    clock.advance(sync_delay)

            robot.update_control = update
            robot.publish_feedback = feedback
            robot.get_logger = lambda: SimpleNamespace(info=log)
            with patch('time.monotonic', clock), patch('time.sleep', sleep), \
                    patch.object(module['rclpy'], 'ok', return_value=True, create=True), \
                    patch.object(module['rclpy'], 'spin_once', spin, create=True), \
                    patch.object(mujoco, 'mj_step', physics), \
                    patch.object(mujoco.viewer, 'launch_passive', return_value=Viewer()):
                robot.run()
            prefix = 'Simulation profile: '
            reports = [json.loads(m[len(prefix):]) for m in messages if m.startswith(prefix)]
            return {'position': robot.joint_positions(), 'velocity': robot.data.qvel.copy(),
                    'target': robot.q_des.copy(), 'command': robot.command.copy(),
                    'trace': trace, 'sleep': sleep_requests, 'reports': reports,
                    'physics_time': robot.data.time, 'wall_time': clock.now - 100.}
        finally:
            fixture.doCleanups()

    def test_profiling_preserves_steps_control_sleep_order_and_feedback_stride(self):
        disabled = self.run_loop(False)
        enabled = self.run_loop(True)
        for name in ['position', 'velocity', 'target', 'command']:
            np.testing.assert_array_equal(enabled[name], disabled[name])
        self.assertEqual(enabled['trace'], disabled['trace'])
        np.testing.assert_allclose(enabled['sleep'], disabled['sleep'], atol=1e-12)
        self.assertEqual(enabled['trace'].count('control'), 20)
        self.assertEqual(enabled['trace'].count('physics'), 20)
        self.assertEqual(enabled['trace'].count('feedback'), 4)
        self.assertEqual(enabled['trace'].count('sync'), 4)
        self.assertAlmostEqual(enabled['physics_time'], .04)
        self.assertEqual(disabled['reports'], [])
        self.assertGreater(len(enabled['reports']), 0)
        for report in enabled['reports']:
            stages = report['stages']
            self.assertAlmostEqual(stages['physics']['mean_ms'], .1)
            self.assertAlmostEqual(stages['viewer_lock_wait']['mean_ms'], .05)
            if stages['viewer_sync']['count']:
                self.assertAlmostEqual(stages['viewer_sync']['mean_ms'], 6.)
                self.assertGreater(stages['viewer_sync']['total_ms'],
                                   stages['physics']['total_ms'])

    def test_injected_viewer_delay_reduces_measured_real_time_factor(self):
        quick = self.run_loop(True, sync_delay=.0002)
        slow = self.run_loop(True, sync_delay=.006)
        self.assertGreater(quick['physics_time'] / quick['wall_time'],
                           slow['physics_time'] / slow['wall_time'])
        self.assertGreater(quick['reports'][0]['real_time_factor'],
                           slow['reports'][0]['real_time_factor'])


if __name__ == '__main__':
    unittest.main()
