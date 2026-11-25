"""Batched RRT markers and freshness gates; no Motion or collision-query API."""

import time

from planning.visualization.rrt_animation import playback_frame, visualization_settings

from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


LAYERS = ('start_tree', 'goal_tree', 'final_path', 'rejected_candidates', 'search_step')


def marker_batches(visualizer, data, historical=False):
    """Use preprojected XYZ only. Lines illustrate joint edges, not Cartesian motion."""
    batches = {}

    def marker(namespace, kind, size, color, points):
        m = visualizer.make_marker(namespace, kind, size, color)
        m.scale.y = size
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        m.lifetime.nanosec = 600_000_000
        return m

    def batch(layer, markers):
        a = MarkerArray()
        a.markers = markers
        batches[layer] = a

    for name, color in (('start', (.1, .4, 1.)), ('goal', (1., .1, .85))):
        nodes = data['trees'][name]
        lines = []
        for i, node in enumerate(nodes):
            parent = node['parent']
            if parent >= 0:
                if parent >= i:
                    raise ValueError('RRT parent must precede its child')
                lines.extend((nodes[parent]['xyz'], node['xyz']))
        namespace = 'rrt_' + name + '_tree'
        batch(name + '_tree', [
            marker(namespace + '_nodes', Marker.POINTS, .009, color,
                   [n['xyz'] for n in nodes]),
            marker(namespace + '_edges', Marker.LINE_LIST, .002, color, lines)])
    batch('final_path', [marker('rrt_final_path', Marker.LINE_STRIP, .008, (.7, .15, 1.),
                                data['path'])])
    # Unknown outcomes are separate from confirmed geometric rejections.
    unknown = {'TIME_BUDGET_EXHAUSTED', 'HUMAN_VISION_UNKNOWN', 'HUMAN_DATA_UNKNOWN',
               'HUMAN_GEOMETRY_UNKNOWN', 'ENVIRONMENT_CHANGED', 'COLLISION_CHECK_ERROR'}
    rejected = data['rejected']
    batch('rejected_candidates', [
        marker('rrt_rejected', Marker.LINE_LIST, .003, (1., .55, 0.),
               [p for r in rejected if r['reason'] not in unknown
                for p in (r['parent_xyz'], r['xyz'])]),
        marker('rrt_unverified', Marker.POINTS, .012, (.7, .7, .7),
               [r['xyz'] for r in rejected if r['reason'] in unknown])])
    step = data.get('step', {})
    batch('search_step', [marker('rrt_search_step', Marker.POINTS, .018, (1., 1., 0.),
                                 [step['xyz']] if 'xyz' in step else [])])
    text = visualizer.make_marker('rrt_status', Marker.TEXT_VIEW_FACING, .025, (.9, .9, .9))
    text.scale.z = .025
    text.pose.position.z = .75
    prefix = ('OFFLINE HISTORICAL REPLAY' if historical else
              'RRT fallback snapshot' if data.get('mode') == 'fallback'
              else 'RRT baseline snapshot')
    prefix = data.get('visual_label', prefix)
    text.text = (f"{prefix}: {data.get('reason', '')}; rev {data.get('scene_revision', 'offline')}"
                 '\nFK joint-tree projection; '
                 'lines are not Cartesian trajectories')
    if step:
        text.text += f"\n{step['event']} {step.get('reason', '')}"
    text.text += (f"\nDisplay omitted: {data.get('dropped_nodes', 0)} nodes, "
                  f"{data.get('dropped_events', 0)} events")
    text.lifetime.nanosec = 600_000_000
    batches['search_step'].markers.append(text)
    return batches


class RRTDisplay:
    def __init__(self, node, visualizer, settings=None):
        self.settings = visualization_settings(settings)
        self.accepted_at = 0.
        self.visualizer = visualizer
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers = {layer: node.create_publisher(
            MarkerArray, '/planning/rrt/' + layer, qos) for layer in LAYERS}
        self.status_pub = node.create_publisher(String, '/planning/rrt/status', qos)
        self.data = self.batches = None
        self.last_publish = float('-inf')
        self.last_status = None
        self.status('WAITING: RRT search visualization; execution owned by Planning')

    def interested(self):
        return any(p.get_subscription_count() for p in self.publishers.values())

    def status(self, text):
        if text != self.last_status:
            self.status_pub.publish(String(data=text))
            self.last_status = text
            marker = self.visualizer.make_marker(
                'rrt_status', Marker.TEXT_VIEW_FACING, .025, (.9, .9, .9))
            marker.scale.z = .025
            marker.pose.position.z = .75
            marker.text = text
            array = MarkerArray()
            array.markers = [marker]
            self.publishers['search_step'].publish(array)

    def clear(self, reason):
        if self.batches is not None:
            for layer, array in self.batches.items():
                for marker in array.markers:
                    marker.action = Marker.DELETE
                self.publishers[layer].publish(array)
        self.data = self.batches = None
        self.status(reason)

    def accept(self, data):
        self.data = data
        self.accepted_at = time.monotonic()
        self.last_publish = float('-inf')

    def tick(self, manager):
        now = time.monotonic()
        if self.data is None:
            return
        checker = manager.planner.collision_filter
        live = self.settings['mode'] == 'live'
        partial = self.data.get('live_partial', False)
        duration = 0. if live else self.settings['playback_duration_s']
        age = now - self.accepted_at
        if manager.target_generation != self.data['target_generation']:
            self.clear('TARGET_CHANGED: old search cleared')
        elif age >= duration + self.settings['history_duration_s']:
            self.clear('HISTORY_FINISHED')
        elif not self.interested():
            self.clear('DISPLAY_DISABLED: no RRT subscribers')
        elif now - self.last_publish >= 1. / self.settings['publish_rate_hz']:
            self.last_publish = now
            fraction = 1. if live else min(1., age / duration)
            frame = playback_frame(self.data, fraction, self.settings['show_rejected'])
            historical = (now >= self.data['expires_at']
                          or checker.environment_revision != self.data['scene_revision']
                          or not manager.obstacle_data_ready()
                          or not manager.rrt_human_ready())
            label = ('LIVE SEARCH' if partial else 'LIVE SEARCH RESULT')
            if not live:
                label = 'SEARCH REPLAY'
            if historical:
                label += ' / HISTORICAL SCENE'
            elapsed = self.data.get('search_elapsed_s')
            if elapsed is not None:
                label += f' / original {elapsed * 1000.:.1f} ms'
            direct = any(e['event'] == 'DIRECT_CONNECTION'
                         for e in self.data.get('events', []))
            if fraction >= 1. and direct:
                label += ' / DIRECT CONNECTION'
            label += f' / Planning: {manager.status} / candidate only'
            frame['visual_label'] = label
            self.batches = marker_batches(self.visualizer, frame)
            for array in self.batches.values():
                for marker in array.markers:
                    marker.header.stamp = self.visualizer.node.get_clock().now().to_msg()
                    lifetime = max(.6, 2. / self.settings['publish_rate_hz'])
                    marker.lifetime.sec = int(lifetime)
                    marker.lifetime.nanosec = int((lifetime % 1.) * 1e9)
                    if historical and marker.type != Marker.TEXT_VIEW_FACING:
                        marker.color.a = .35
            for layer, array in self.batches.items():
                self.publishers[layer].publish(array)
            self.status_pub.publish(String(data=label))
