"""Bounded RRT presentation policy, independent of collision and Motion logic."""

import math

from planning.rrt_trace import SearchTrace


DEFAULTS = dict(mode='playback', publish_rate_hz=10., playback_duration_s=2.,
                show_rejected=False, max_nodes_per_tree=256, max_events=512,
                history_duration_s=3.)


def visualization_settings(settings=None):
    result = dict(DEFAULTS)
    if settings is not None:
        if not isinstance(settings, dict) or set(settings) - set(DEFAULTS):
            raise ValueError('Unknown RRT visualization setting')
        result.update(settings)
    if result['mode'] not in ('live', 'playback'):
        raise ValueError('RRT visualization mode must be live or playback')
    if type(result['show_rejected']) is not bool:
        raise ValueError('show_rejected must be boolean')
    for key, upper in (('max_nodes_per_tree', 1024), ('max_events', 4096)):
        if type(result[key]) is not int or not 2 <= result[key] <= upper:
            raise ValueError(f'{key} must be an integer from 2 to {upper}')
    for key, upper in (('publish_rate_hz', 20.), ('playback_duration_s', 30.),
                       ('history_duration_s', 30.)):
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f'{key} must be positive and at most {upper}')
    return result


def playback_frame(data, fraction, show_rejected=False):
    """Reveal recorded events only; never fabricate branches for direct routes."""
    events = data.get('events', [])
    count = max(1, min(len(events), math.ceil(len(events) * fraction)))
    complete = fraction >= 1.
    trees = {name: [n for n in nodes if complete or n.get('event_index', 0) < count]
             for name, nodes in data['trees'].items()}
    rejected = [r for r in data.get('rejected', [])
                if complete or r.get('event_index', 0) < count] if show_rejected else []
    step = events[min(count, len(events)) - 1] if events else {}
    return dict(data, trees=trees, rejected=rejected,
                events=events[:count], step=step, path=data['path'] if complete else [])


class LiveProjection:
    """Separate FK data and a bounded per-attempt cache; no collision queries."""

    def __init__(self, checker, bodies):
        import mujoco
        self.model = checker.model
        self.data = mujoco.MjData(self.model)
        self.addresses = checker.qpos_addresses
        self.base, self.eof = (self.model.body(name).id for name in bodies)
        self.attempt = None
        self.cache = {}

    def project(self, packet):
        from planning.visualization.projection import project_configuration
        if self.attempt != packet['attempt']:
            self.cache.clear()
            self.attempt = packet['attempt']

        def point(q):
            key = tuple(q)
            if key not in self.cache:
                self.cache[key] = project_configuration(
                    self.model, self.data, self.addresses, self.base, self.eof, q).tolist()
            return self.cache[key]

        record = SearchTrace()
        for key in ('trees', 'events', 'rejected', 'path', 'dropped_nodes', 'dropped_events'):
            setattr(record, key, packet[key])
        projected = record.project(point)
        return dict(packet, **projected)
