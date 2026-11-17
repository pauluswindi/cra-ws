"""Verify diagnostic passivity, absolute expiry and acknowledgement accounting."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

from planning.profiling import PlanningProfiler


class PlanningProfilingTests(unittest.TestCase):
    def make_profiler(self, enabled=True):
        checker = SimpleNamespace(
            human_status='OBSERVED', human_received_at=100.,
            human_expires_at=100.75, human_data_timeout=1.)
        return PlanningProfiler(checker, enabled)

    def test_work_crossing_capture_deadline_does_not_refresh_it(self):
        profiler = self.make_profiler()
        before = vars(profiler.checker).copy()
        marker = object()
        with patch('time.monotonic', side_effect=[100.5, 100.9]), \
                patch('time.perf_counter', side_effect=[10., 10.4]):
            self.assertIs(profiler.call('validation', lambda: marker), marker)
        row = profiler.records[-1]
        self.assertEqual(row['remaining_before_ms'], 250.)
        self.assertEqual(row['remaining_after_ms'], -150.)
        self.assertAlmostEqual(row['wall_ms'], 400.)
        self.assertEqual(vars(profiler.checker), before)

    def test_failed_check_and_exception_keep_original_results(self):
        profiler = self.make_profiler()
        self.assertFalse(profiler.call('collision', lambda: False))
        self.assertEqual(profiler.records[-1]['outcome'], 'FALSE')
        error = ValueError('certificate expired')

        def fail():
            raise error

        with self.assertRaises(ValueError) as caught:
            profiler.call('certificate', fail)
        self.assertIs(caught.exception, error)
        self.assertEqual(profiler.records[-1]['outcome'], 'ERROR')

    def test_disabled_profiler_does_not_read_clocks_or_checker(self):
        profiler = PlanningProfiler(None, enabled=False)
        with patch('time.monotonic', side_effect=AssertionError('unexpected clock')), \
                patch('time.perf_counter', side_effect=AssertionError('unexpected clock')):
            self.assertEqual(profiler.call('search', lambda: 17), 17)
        self.assertFalse(profiler.records)

    def test_release_ack_survives_next_cycle_and_cancel_does_not_misattribute(self):
        profiler = self.make_profiler()
        with patch('time.monotonic', return_value=100.):
            profiler.start_cycle()
            profiler.command_sent('RELEASE', 100.)
        with patch('time.monotonic', return_value=100.3):
            profiler.command_response('RELEASE', 'IDLE')
        profiler.start_cycle()
        row = profiler.records[-1]
        self.assertEqual(row['stage'], 'release_ack_wait')
        self.assertAlmostEqual(row['wall_ms'], 300.)
        self.assertEqual(row['origin_cycle'], 1)
        self.assertEqual(row['remaining_before_ms'], 750.)
        self.assertEqual(row['remaining_after_ms'], 450.)
        profiler.command_sent('RELEASE', 100.3)
        profiler.command_response('RELEASE', 'STOPPED')
        profiler.command_response(None, 'IDLE')
        profiler.start_cycle()
        self.assertFalse(profiler.records)

    def test_diagnostics_are_bounded_and_unknown_has_no_budget(self):
        profiler = self.make_profiler()
        profiler.checker.human_status = 'UNKNOWN'
        for _ in range(100):
            profiler.call('check', lambda: None)
        self.assertEqual(len(profiler.records), 64)
        self.assertIsNone(profiler.records[-1]['remaining_before_ms'])
        self.assertIsNone(profiler.records[-1]['remaining_after_ms'])
        profiler.start_cycle()
        self.assertFalse(profiler.records)


class PlanningProfileNodeTests(unittest.TestCase):
    """Exercise actual node methods with inert publishers, without importing ROS."""

    @classmethod
    def setUpClass(cls):
        source = Path(__file__).resolve().parents[1] / 'planning/planning_node.py'
        node = next(n for n in ast.parse(source.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == 'PlanningNode')
        methods = [n for n in node.body if isinstance(n, ast.FunctionDef)
                   and n.name in (
                       'planning_cycle', 'report_planning_profile', 'report_path_failure')]
        namespace = {'json': json, 'time': time, 'Bool': SimpleNamespace}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
        cls.cycle = staticmethod(namespace['planning_cycle'])
        cls.report = staticmethod(namespace['report_planning_profile'])
        cls.failure_report = staticmethod(namespace['report_path_failure'])

    def test_motion_dispatch_precedes_visualization_and_profile_logging(self):
        for action in ('STOP', 'RELEASE', 'EXECUTE'):
            with self.subTest(action=action):
                events = []
                payloads = []
                failures = []
                checker = SimpleNamespace(human_status='UNKNOWN', human_data_timeout=1.)
                profiler = PlanningProfiler(checker, enabled=True)
                profiler.start_cycle()
                decision = SimpleNamespace(action=action, status='TEST', path=[[0.] * 5])
                manager = SimpleNamespace(
                    profiler=profiler, human_status='UNKNOWN', human_reason='NO_DATA',
                    human_path_validation_reason='NONE', step=lambda: decision,
                    path_failure_sequence=1, execution_path_id=1,
                    execution_next_waypoint=1, _candidate_next_waypoint=lambda: 1,
                    execution_progress_reason='COMMAND_SENT',
                    segment_progress=None,
                    last_path_failure={'id': 1, 'failure': {'check': 'WAYPOINT'}})

                def log(message):
                    if message.startswith('Planning profile: '):
                        events.append('log')
                        payloads.append(json.loads(message.removeprefix('Planning profile: ')))
                    elif message.startswith('Path validation failure: '):
                        events.append('failure_log')
                        failures.append(json.loads(
                            message.removeprefix('Path validation failure: ')))

                node = SimpleNamespace(
                    manager=manager, planner=SimpleNamespace(collision_filter=checker),
                    human_monitor=SimpleNamespace(stamp_ns=None),
                    last_human_callback_profile=None, last_profile_report_time=None,
                    last_profile_decision=None, monitor_humans=lambda: None,
                    report_collision_diagnostics=lambda: None, set_status=lambda _: None,
                    report_runtime_diagnostics=lambda: None,
                    last_path_failure_id=0,
                    update_visualization=lambda: events.append('visualization'),
                    update_graph_visualization=lambda: None,
                    build_trajectory=lambda _: SimpleNamespace(points=[1]),
                    stop_pub=SimpleNamespace(publish=lambda _: events.append('dispatch')),
                    motion_pub=SimpleNamespace(publish=lambda _: events.append('dispatch')),
                    get_logger=lambda: SimpleNamespace(info=log),
                )
                node.report_planning_profile = lambda *args: self.report(node, *args)
                node.report_path_failure = lambda: self.failure_report(node)
                self.cycle(node)
                self.assertLess(events.index('dispatch'), events.index('visualization'))
                self.assertLess(events.index('dispatch'), events.index('log'))
                self.assertLess(events.index('dispatch'), events.index('failure_log'))
                self.assertEqual(payloads[-1]['action'], action)
                self.assertIsNone(payloads[-1]['capture_age_start_ms'])
                self.assertEqual(failures, [manager.last_path_failure])
                node.report_path_failure()
                self.assertEqual(len(failures), 1)


if __name__ == '__main__':
    unittest.main()
