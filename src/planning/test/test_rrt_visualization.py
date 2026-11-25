"""Search certificates, bounded replay, shared FK and observational marker gates."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
import numpy as np

from benchmark_rrt_connect import FAR, GOAL, GRAPH, START, configure, geom_center, planner
from planning.algorithms.rrt_connect import RRTConnectPlanner
from planning.human_capsule_monitor import HumanCapsule, HumanSnapshot
from planning.rrt_replay import replay_frames
from planning.rrt_shadow import ShadowRRTObserver, capture_scene, run_job
from planning.rrt_trace import SearchTrace
from planning.visualization.planning_visualizer import PlanningVisualizer
from planning.visualization.projection import project_configuration
from planning.visualization.rrt_display import RRTDisplay, marker_batches
from test_rrt_connect import WallChecker, q
from test_rrt_shadow import ManualExecutor, manager_for, options


_NATIVE_PLANNER = None


def native_planner():
    # MuJoCo 3.1.6 retains STL descriptors across model loads. Share this
    # scheduling/marker fixture instead of exhausting the suite's OS limit.
    global _NATIVE_PLANNER
    if _NATIVE_PLANNER is None:
        _NATIVE_PLANNER = planner(FAR)
    configure(_NATIVE_PLANNER.collision_filter, FAR)
    return _NATIVE_PLANNER


class TraceTests(unittest.TestCase):
    def search(self, **limits):
        checker = WallChecker()
        trace = SearchTrace(**limits)
        search = RRTConnectPlanner(checker, clock=lambda: 0., trace=trace)
        path = search.plan(q(-.8, -.6), q(.8, -.6), seed=9)
        return checker, trace, search, path

    def test_trace_does_not_change_path_checks_or_sampling(self):
        checker, trace, search, path = self.search(max_events=4096)
        plain = RRTConnectPlanner(WallChecker(), clock=lambda: 0.)
        np.testing.assert_array_equal(path, plain.plan(q(-.8, -.6), q(.8, -.6), seed=9))
        self.assertEqual(search.last_statistics, plain.last_statistics)
        self.assertTrue(trace.rejected)
        for name, nodes in trace.trees.items():
            for i, node in enumerate(nodes):
                self.assertTrue(checker.is_state_valid(node['q']))
                parent = node['parent']
                if parent >= 0:
                    self.assertLess(parent, i)
                    edge = (nodes[parent]['q'], node['q'])
                    if name == 'goal':
                        edge = edge[::-1]
                    self.assertIn(tuple(tuple(v) for v in edge), checker.directed_certificates)
        self.assertEqual(trace.events[0]['event'], 'SEARCH_STARTED')
        self.assertEqual(trace.events[-1]['event'], 'PATH_FOUND')
        events = {e['event'] for e in trace.events}
        self.assertTrue({'SAMPLE_GENERATED', 'NEAREST_SELECTED', 'CONNECT_STARTED',
                         'CONNECT_ADVANCED', 'CONNECT_REACHED'}.issubset(events))

    def test_direct_edge_rejection_does_not_become_accepted_tree_edge(self):
        checker, trace, _, _ = self.search()
        first = trace.rejected[0]
        self.assertEqual(first['check'], 'edge')
        self.assertFalse(checker.is_edge_valid(first['parent_q'], first['q']))
        self.assertTrue(checker.is_state_valid(first['q']))
        self.assertTrue(any(r['check'] == 'state' for r in trace.rejected))

    def test_small_display_caps_preserve_success_and_parent_indices(self):
        _, trace, _, path = self.search(max_nodes=2, max_events=8, max_rejected=2)
        self.assertIsNotNone(path)
        self.assertEqual(len(trace.events), 8)
        self.assertGreater(trace.dropped_events, 0)
        self.assertGreater(trace.dropped_nodes, 0)
        self.assertTrue(all(len(n) <= 2 for n in trace.trees.values()))
        data = dict(trace.project(lambda q: q[:3]), reason='NONE')
        for frame in replay_frames(data):
            self.assertTrue(all(len(n) <= 2 for n in frame['trees'].values()))
            self.assertLessEqual(len(frame['rejected']), 2)

    def test_replay_order_and_no_final_path_before_certificate(self):
        _, trace, _, _ = self.search()
        data = dict(trace.project(lambda q: q[:3]), reason='NONE')
        frames = list(replay_frames(data))
        self.assertEqual([f['step']['event'] for f in frames],
                         [e['event'] for e in trace.events])
        self.assertTrue(all(not f['path'] for f in frames[:-1]))
        self.assertEqual(frames[-1]['trees'], data['trees'])
        self.assertTrue(frames[-1]['path'])

    def test_unknown_during_check_is_not_labelled_geometric_collision(self):
        checker = WallChecker()
        checker.human_obstacles_enabled = True
        checker.on_edge = lambda: setattr(checker, 'ready', False)
        trace = SearchTrace()
        search = RRTConnectPlanner(checker, clock=lambda: 0., trace=trace)
        self.assertIsNone(search.plan(q(-.8, .7), q(.8, .7)))
        self.assertEqual(trace.rejected[0]['reason'], 'HUMAN_VISION_UNKNOWN')
        self.assertFalse(trace.path)
        self.assertEqual(trace.events[-1]['event'], 'SEARCH_FAILED')

    def test_trace_reset_between_searches(self):
        _, trace, search, _ = self.search()
        search.plan(q(-.8, .7), q(.8, .7))
        self.assertEqual(len(trace.trees['start']), 1)
        self.assertFalse(trace.rejected)

    def test_projection_only_once_per_distinct_configuration(self):
        _, trace, _, _ = self.search()
        calls = []

        def project(q):
            calls.append(tuple(q))
            return q[:3]

        trace.project(project)
        self.assertEqual(len(calls), len(set(calls)))


class FakeNode:
    def __init__(self):
        self.messages = {}

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

    def create_publisher(self, kind, topic, qos):
        self.messages[topic] = []
        return SimpleNamespace(publish=self.messages[topic].append,
                               get_subscription_count=lambda: 1)


class MarkerGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = native_planner()

    def setUp(self):
        self.p.collision_filter.enable_human_obstacles(True)
        self.p.collision_filter.set_human_capsules(FAR)
        self.manager = manager_for(self.p)
        self.manager.status = 'IDLE'
        self.manager.rrt_human_ready = lambda: (
            self.manager.human_status == 'EMPTY' or self.p.collision_filter.human_data_ready())
        self.node = FakeNode()
        self.visualizer = object.__new__(PlanningVisualizer)
        self.visualizer.node = self.node
        self.visualizer.frame_id = 'base_link'
        self.display = RRTDisplay(self.node, self.visualizer)
        self.data = dict(trees={
            'start': [dict(xyz=[0., 0., 0.], parent=-1),
                      dict(xyz=[1., 0., 0.], parent=0)],
            'goal': [dict(xyz=[2., 0., 0.], parent=-1)]},
            path=[[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]], rejected=[],
            scene_revision=self.p.collision_filter.environment_revision,
            expires_at=float('inf'), target_generation=1, reason='NONE')

    def test_batched_markers_preserve_parent_edge_and_frame(self):
        batches = marker_batches(self.visualizer, self.data)
        start = batches['start_tree'].markers
        self.assertEqual(len(start[0].points), 2)
        self.assertEqual([p.x for p in start[1].points], [0., 1.])
        self.assertEqual(start[0].header.frame_id, 'base_link')
        self.assertEqual(start[0].lifetime.nanosec, 600_000_000)
        self.assertEqual(len(batches['final_path'].markers[0].points), 3)

    def test_scene_change_keeps_labeled_history_without_command(self):
        self.display.accept(self.data)
        self.p.collision_filter.environment_revision += 1
        self.display.tick(self.manager)
        self.assertIsNotNone(self.display.data)
        label = self.node.messages['/planning/rrt/status'][-1].data
        self.assertIn('HISTORICAL SCENE', label)
        self.assertIn('candidate only', label)
        self.assertIsNone(self.manager.pending_action)

    def test_expired_scene_is_history_not_a_refreshed_certificate(self):
        self.data['expires_at'] = 0.
        self.display.accept(self.data)
        self.display.tick(self.manager)
        self.assertEqual(self.display.data['expires_at'], 0.)
        self.assertIn('HISTORICAL SCENE', self.node.messages['/planning/rrt/status'][-1].data)

    def test_history_has_fixed_expiry_independent_of_human_updates(self):
        with patch('time.monotonic', return_value=100.):
            self.display.accept(self.data)
        with patch('time.monotonic', return_value=105.):
            self.display.tick(self.manager)
        self.assertIsNone(self.display.data)
        self.assertEqual(self.display.last_status, 'HISTORY_FINISHED')

    def test_target_replacement_clears_geometry(self):
        self.display.accept(self.data)
        self.manager.target_generation += 1
        self.display.tick(self.manager)
        self.assertIsNone(self.display.data)
        self.assertIsNone(self.manager.pending_action)

    def test_unknown_rejection_has_distinct_gray_layer(self):
        self.data['rejected'] = [dict(reason='HUMAN_VISION_UNKNOWN',
                                      xyz=[1., 0., 0.], parent_xyz=[0., 0., 0.])]
        markers = marker_batches(self.visualizer, self.data)['rejected_candidates'].markers
        self.assertFalse(markers[0].points)
        self.assertEqual(len(markers[1].points), 1)

    def test_fresh_empty_fallback_snapshot_is_visible_until_original_expiry(self):
        self.manager.human_status = 'EMPTY'
        self.manager.human_empty_until = 102.
        self.p.collision_filter.human_received_at = None
        self.data.update(human_data_kind='EMPTY', mode='fallback', expires_at=102.)
        with patch('time.monotonic', return_value=100.):
            self.display.accept(self.data)
        with patch('time.monotonic', return_value=100.):
            self.display.tick(self.manager)
        self.assertIsNotNone(self.display.data)
        self.assertTrue(self.node.messages['/planning/rrt/start_tree'][-1].markers[0].points)
        with patch('time.monotonic', return_value=102.1):
            self.display.tick(self.manager)
        self.assertIsNotNone(self.display.data)
        self.assertIn('HISTORICAL SCENE', self.node.messages['/planning/rrt/status'][-1].data)


class NativeProjectionTests(unittest.TestCase):
    def setUp(self):
        self.p = native_planner()

    def test_projection_matches_stored_workspace_joint_order_and_frame(self):
        c = self.p.collision_filter
        with np.load(GRAPH, allow_pickle=False) as graph:
            for i in (0, 17, 100, 500, 1000):
                projected = project_configuration(c.model, c.data, c.qpos_addresses,
                                                  c.base_id, c.model.body('end_effector').id,
                                                  graph['joint_q'][i])
                np.testing.assert_allclose(projected, graph['joint_eof'][i], atol=1e-6)
        with self.assertRaises(ValueError):
            project_configuration(c.model, c.data, c.qpos_addresses, c.base_id, 0,
                                  [float('nan')] * 5)

    def test_real_worker_projects_without_changing_result(self):
        with patch('time.monotonic', return_value=100.):
            p = self.p
            configure(p.collision_filter, FAR)
            job = dict(kind='search', scene=capture_scene(p.collision_filter), sample_step=.01,
                       search_budget=2., selection_budget=3., preference_search_budget=.8,
                       total_budget=5., preferred_clearance=.07, start=START, goal=GOAL, seed=9,
                       visualize=True)
            result = run_job(job, p.collision_filter)
            self.assertEqual(result['reason'], 'NONE')
            data = result['visualization']
            self.assertTrue(data['path'])
            self.assertEqual(data['scene_revision'], job['scene']['revision'])
            self.assertEqual(data['expires_at'], 102.)
            self.assertEqual(len(data['trees']['start']), 1)

    def test_actual_human_blocks_direct_edge_without_inserting_it(self):
        with patch('time.monotonic', return_value=100.):
            c = self.p.collision_filter
            center = geom_center(c, (START + GOAL) / 2., 'link4_visual')
            blocked = HumanSnapshot('OBSERVED', 'CAPSULES_PRESENT', (
                HumanCapsule(0, 0, tuple(center), tuple(center + [0., 0., .08]), .05),))
            configure(c, blocked)
            self.assertTrue(c.is_state_valid(START))
            self.assertTrue(c.is_state_valid(GOAL))
            self.assertFalse(c.is_edge_valid(START, GOAL))
            trace = SearchTrace()
            search = RRTConnectPlanner(c, trace=trace, time_budget=5.)
            path = search.plan(START, GOAL, seed=9)
            self.assertIsNotNone(path)
            self.assertEqual(trace.rejected[0]['reason'], 'HUMAN_COLLISION')
            for nodes in trace.trees.values():
                for node in nodes:
                    self.assertTrue(c.is_state_valid(node['q']))
            self.assertTrue(c.is_path_valid(path))


class ShadowTraceHandoffTests(unittest.TestCase):
    def test_trace_is_opt_in_and_never_serialized_into_shadow_status(self):
        p = native_planner()
        manager = manager_for(p)
        executor = ManualExecutor()
        observer = ShadowRRTObserver('unused', p.joint_names,
                                     options(p.collision_filter), executor=executor)
        self.addCleanup(observer.close)
        observer.tick(manager)
        self.assertFalse(executor.jobs[-1][1]['visualize'])
        trace = dict(trees={'start': [], 'goal': []}, scene_revision=1, events=[],
                     projection_s=0., dropped_nodes=0, dropped_events=0)
        executor.jobs[-1][2].set_result(dict(
            kind='search', reason='TIME_BUDGET_EXHAUSTED', path=None, visualization=trace))
        event = observer.tick(manager)
        self.assertNotIn('visualization', event)
        self.assertEqual(observer.visualization_result['trees'], trace['trees'])
        self.assertIsNone(manager.pending_action)

    def test_renderer_exception_does_not_escape_observation_callback(self):
        from planning.planning_node import PlanningNode

        errors, clears = [], []

        def fail(_):
            raise ValueError('bad projection')

        node = SimpleNamespace(
            rrt_visualization_enabled=True, manager=SimpleNamespace(pending_action=None),
            rrt_shadow=SimpleNamespace(tick=lambda _: None, visualization_result={'bad': True}),
            rrt_display=SimpleNamespace(interested=lambda: True, accept=fail, clear=clears.append),
            get_logger=lambda: SimpleNamespace(error=errors.append))
        PlanningNode.observe_rrt(node)
        self.assertTrue(errors and clears)
        self.assertIsNone(node.manager.pending_action)


if __name__ == '__main__':
    unittest.main()
