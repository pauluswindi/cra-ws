"""Bounded observational data; no checker queries or planner decisions."""

import numpy as np


class SearchTrace:
    def __init__(self, max_nodes=256, max_events=512, max_rejected=64):
        if any(type(v) is not int or v < 2 for v in (max_nodes, max_events, max_rejected)):
            raise ValueError('Trace limits must be integers >= 2')
        self.max_nodes, self.max_events, self.max_rejected = (
            max_nodes, max_events, max_rejected)
        self.reset()

    def reset(self):
        self.trees = {'start': [], 'goal': []}
        self.events, self.rejected = [], []
        self.dropped_events = self.dropped_nodes = 0
        self.path = []

    def emit(self, event, **fields):
        record = dict(event=event, **fields)
        if len(self.events) < self.max_events - 1:
            self.events.append(record)
        else:
            self.dropped_events += 1

    def node(self, tree, q, parent=-1, iteration=0):
        nodes = self.trees[tree]
        visible = len(nodes) < self.max_nodes
        if visible:
            nodes.append(dict(q=np.asarray(q).tolist(), parent=int(parent)))
        else:
            self.dropped_nodes += 1
        self.emit('EXTEND_ACCEPTED', tree=tree, q=np.asarray(q).tolist(),
                  parent=int(parent), iteration=iteration, displayed=visible)

    def reject(self, tree, q, parent_q, reason, check, iteration):
        record = dict(tree=tree, q=np.asarray(q).tolist(),
                      parent_q=np.asarray(parent_q).tolist(), reason=reason,
                      check=check, iteration=iteration)
        visible = len(self.rejected) < self.max_rejected
        if visible:
            self.rejected.append(record)
        self.emit('EXTEND_REJECTED', displayed=visible, **record)

    def finish(self, path, reason):
        self.path = [] if path is None else np.asarray(path).tolist()
        self.events.append(dict(event='PATH_FOUND' if path is not None else 'SEARCH_FAILED',
                                reason=reason))

    def project(self, project):
        """Project each distinct configuration once, after the search has ended."""
        cache = {}

        def point(q):
            key = tuple(q)
            if key not in cache:
                cache[key] = np.asarray(project(q), dtype=float).tolist()
            return cache[key]

        trees = {name: [dict(xyz=point(n['q']), parent=n['parent']) for n in nodes]
                 for name, nodes in self.trees.items()}
        rejected = [dict(r, xyz=point(r['q']), parent_xyz=point(r['parent_q']))
                    for r in self.rejected]
        events = []
        for event in self.events:
            projected = dict(event)
            for source, target in (('q', 'xyz'), ('parent_q', 'parent_xyz')):
                if source in event:
                    projected[target] = point(event[source])
            events.append(projected)
        return dict(trees=trees, rejected=rejected, events=events,
                    path=[point(q) for q in self.path], dropped_events=self.dropped_events,
                    dropped_nodes=self.dropped_nodes, projection='FK joint-tree projection')
