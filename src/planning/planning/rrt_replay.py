"""Replay a bounded recorded search on marker topics only; never loads Motion."""

import argparse
import copy
import json
from pathlib import Path
import time


def replay_frames(data):
    """Build tree prefixes from recorded events; samples never become tree nodes."""
    frame = dict(trees={'start': [], 'goal': []}, rejected=[], path=[],
                 reason=data['reason'], dropped_nodes=data['dropped_nodes'],
                 dropped_events=data['dropped_events'])
    for event in data['events']:
        kind = event['event']
        frame['step'] = event
        if kind == 'EXTEND_ACCEPTED' and event.get('displayed', True):
            frame['trees'][event['tree']].append(
                dict(xyz=event['xyz'], parent=event['parent']))
        elif kind == 'EXTEND_REJECTED' and event.get('displayed', True):
            frame['rejected'].append(event)
        elif kind in ('PATH_FOUND', 'SEARCH_FAILED'):
            # Event capping may omit growth; the terminal frame uses the bounded
            # completed snapshot explicitly, without inventing missing events.
            frame = copy.deepcopy(data)
            frame['step'] = event
            if kind != 'PATH_FOUND':
                frame['path'] = []
        yield copy.deepcopy(frame)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=5.)
    args = parser.parse_args()
    if not 0.1 <= args.rate <= 20.:
        parser.error('rate must be in [0.1, 20] events/second')
    data = json.loads(args.file.read_text())

    import rclpy
    from rclpy.node import Node
    from planning.visualization.planning_visualizer import PlanningVisualizer
    from planning.visualization.rrt_display import RRTDisplay, marker_batches

    rclpy.init()
    node = Node('rrt_offline_replay')
    # Marker creation needs a clock/frame only, not workspace assets or MuJoCo.
    visualizer = object.__new__(PlanningVisualizer)
    visualizer.node = node
    visualizer.frame_id = data['frame_id']
    display = RRTDisplay(node, visualizer)
    frames = iter(replay_frames(data))
    last = [None]
    next_step = [time.monotonic()]

    def publish():
        if time.monotonic() >= next_step[0]:
            last[0] = next(frames, last[0])
            next_step[0] += 1. / args.rate
        if last[0] is not None:
            for layer, array in marker_batches(visualizer, last[0], historical=True).items():
                display.publishers[layer].publish(array)

    node.create_timer(min(.2, 1. / args.rate), publish)
    node.get_logger().info('OFFLINE HISTORICAL REPLAY; use an isolated ROS_DOMAIN_ID')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        display.clear('Replay stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
