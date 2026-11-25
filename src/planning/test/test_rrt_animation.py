"""Presentation modes, bounded trace capture and observational ownership."""

import unittest

from planning.rrt_trace import SearchTrace
from planning.visualization.rrt_animation import playback_frame, visualization_settings
import test_rrt_visualization as visual_tests


class AnimationTests(unittest.TestCase):
    def test_settings_modes_and_bounds(self):
        self.assertEqual(visualization_settings()['mode'], 'playback')
        self.assertEqual(visualization_settings({'mode': 'live'})['mode'], 'live')
        for invalid in ({'mode': 'slow'}, {'max_events': 1000000},
                        {'publish_rate_hz': 100}, {'show_rejected': 'false'},
                        {'history_duration_s': float('nan')}, {'oops': 1}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                visualization_settings(invalid)

    def test_playback_reveals_real_nodes_and_only_later_the_path(self):
        _, trace, _, _ = visual_tests.TraceTests().search()
        data = trace.project(lambda q: q[:3])
        early = playback_frame(data, .01)
        late = playback_frame(data, 1.)
        self.assertFalse(early['path'])
        self.assertLess(sum(map(len, early['trees'].values())),
                        sum(map(len, late['trees'].values())))
        self.assertEqual(late['path'], data['path'])
        self.assertFalse(late['rejected'])
        self.assertTrue(playback_frame(data, 1., True)['rejected'])

    def test_broken_visual_transport_cannot_break_search(self):
        trace = SearchTrace(progress=lambda _: (_ for _ in ()).throw(RuntimeError()))
        trace.emit('SEARCH_STARTED')
        trace.emit('NEXT')
        self.assertEqual(len(trace.events), 2)
        self.assertIsNone(trace.progress)


class NativeProgressTests(unittest.TestCase):
    def test_full_visual_queue_never_blocks_or_changes_real_search(self):
        import queue
        from unittest.mock import patch
        from benchmark_rrt_connect import START, GOAL
        from planning.rrt_shadow import run_job, capture_scene
        from test_rrt_visualization import native_planner
        checker = native_planner().collision_filter
        transport = queue.Queue(maxsize=1)
        transport.put('already full')
        scene = capture_scene(checker)
        job = dict(kind='search', scene=scene, sample_step=.01, search_budget=1.,
                   start=START, goal=GOAL, seed=0, total_budget=1.,
                   use_preference=False, visualize=True, visual_attempt=1, visual_generation=1,
                   visualization_settings=visualization_settings({'mode': 'live'}))
        with patch('planning.rrt_shadow._PROGRESS', transport):
            result = run_job(job, checker)
        self.assertEqual(result['reason'], 'NONE')
        self.assertEqual(transport.get_nowait(), 'already full')
        self.assertIn('visualization', result)

    def test_live_projection_has_private_data_and_reuses_fk_cache(self):
        import numpy as np
        from benchmark_rrt_connect import START, GOAL
        from planning.visualization.rrt_animation import LiveProjection
        from test_rrt_visualization import native_planner
        checker = native_planner().collision_filter
        before = checker.data.qpos.copy()
        record = SearchTrace()
        record.node('start', START)
        record.node('goal', GOAL)
        packet = dict(attempt=1, trees=record.trees, events=record.events,
                      rejected=[], path=[], dropped_nodes=0, dropped_events=0)
        projection = LiveProjection(checker, ('base_link', 'end_effector'))
        first = projection.project(packet)
        self.assertEqual(len(projection.cache), 2)
        self.assertEqual(first, projection.project(packet))
        self.assertEqual(len(projection.cache), 2)
        np.testing.assert_array_equal(checker.data.qpos, before)


if __name__ == '__main__':
    unittest.main()
