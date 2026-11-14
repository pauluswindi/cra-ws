"""Check lazy A* certificates, fallbacks and snapshot/cache ownership."""

import contextlib
import heapq
import io
import unittest

import numpy as np

from planning.algorithms.a_star import AStarPlanner


def q(x, y=0.):
    return np.array([x, y, 0., 0., 0.])


def edge_key(a, b):
    return tuple(sorted((tuple(a), tuple(b))))


class Checker:
    def __init__(self):
        self.environment_revision = 0
        self.human_obstacles_enabled = False
        self.ready = True
        self.blocked_states = set()
        self.blocked_edges = set()
        self.certified_edges = set()
        self.on_edge = None

    def human_data_ready(self):
        return self.ready

    def is_state_valid(self, configuration):
        return tuple(configuration) not in self.blocked_states

    def is_edge_valid(self, a, b):
        if self.on_edge is not None:
            self.on_edge()
        key = edge_key(a, b)
        valid = (key not in self.blocked_edges
                 and self.is_state_valid(a) and self.is_state_valid(b))
        if valid:
            self.certified_edges.add(key)
        return valid


class AStarResponsivenessTests(unittest.TestCase):
    def setUp(self):
        self.checker = Checker()
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def planner(self, configurations=None, edges=None):
        if configurations is None:
            configurations = np.array([q(0), q(1, .2), q(1, -1), q(2)])
        if edges is None:
            edges = np.array([[0, 1], [1, 3], [0, 2], [2, 3]])
        costs = np.linalg.norm(configurations[edges[:, 0]] - configurations[edges[:, 1]], axis=1)
        return AStarPlanner(configurations, edges, costs, self.checker,
                            max_joint_delta=.25, connection_count=1)

    def assert_certified(self, path):
        self.assertIsNotNone(path)
        for configuration in path:
            self.assertTrue(self.checker.is_state_valid(configuration))
        for a, b in zip(path[:-1], path[1:]):
            self.assertIn(edge_key(a, b), self.checker.certified_edges)
            self.assertNotIn(edge_key(a, b), self.checker.blocked_edges)

    def test_first_blocked_edge_is_rejected_and_other_branch_is_certified(self):
        planner = self.planner()
        self.checker.blocked_edges.add(edge_key(q(0), q(1, .2)))
        path = planner.plan(q(0), q(2))
        self.assert_certified(path)
        self.assertTrue(any(np.array_equal(point, q(1, -1)) for point in path))
        self.assertGreaterEqual(planner.last_candidate_count, 2)
        self.assertAlmostEqual(np.linalg.norm(np.diff(path, axis=0), axis=1).sum(), 2 * np.sqrt(2))

    def test_blocked_state_is_removed_from_subsequent_candidates(self):
        planner = self.planner()
        self.checker.blocked_states.add(tuple(q(1, .2)))
        path = planner.plan(q(0), q(2))
        self.assert_certified(path)
        self.assertFalse(any(np.array_equal(point, q(1, .2)) for point in path))

    def test_no_path_is_returned_if_both_graph_routes_are_blocked(self):
        planner = self.planner()
        self.checker.blocked_edges.update([
            edge_key(q(0), q(1, .2)), edge_key(q(0), q(1, -1))])
        self.assertIsNone(planner.plan(q(0), q(2)))

    def test_existing_local_connection_fallback_preserves_a_valid_frontier(self):
        configurations = np.array([q(0), q(.2), q(.4), q(.6)])
        planner = self.planner(configurations, np.array([[0, 1], [2, 3]]))
        path = planner.plan(q(0), q(.6))
        self.assert_certified(path)
        self.assertTrue(planner.last_used_local_connections)
        self.assertIn(edge_key(q(.2), q(.4)), self.checker.certified_edges)

    def test_expiry_during_edge_check_does_not_return_or_poison_a_path(self):
        planner = self.planner()
        self.checker.human_obstacles_enabled = True
        self.checker.on_edge = lambda: setattr(self.checker, 'ready', False)
        self.assertIsNone(planner.plan(q(0), q(2)))
        self.assertNotIn(False, planner.edge_cache.values())
        self.checker.on_edge = None
        self.checker.ready = True  # Fresh identical geometry, same revision.
        self.assert_certified(planner.plan(q(0), q(2)))

    def test_geometry_revision_during_validation_cannot_return_a_path(self):
        planner = self.planner()
        self.checker.on_edge = lambda: setattr(self.checker, 'environment_revision', 1)
        with self.assertRaises(RuntimeError):
            planner.plan(q(0), q(2))

    def test_new_geometry_invalidates_old_blocked_edge_cache(self):
        planner = self.planner()
        self.checker.blocked_edges.add(edge_key(q(0), q(1, .2)))
        self.assert_certified(planner.plan(q(0), q(2)))
        self.checker.blocked_edges.clear()
        self.checker.environment_revision += 1
        path = planner.plan(q(0), q(2))
        self.assert_certified(path)
        self.assertTrue(any(np.array_equal(point, q(1, .2)) for point in path))

    def test_new_temporary_start_is_revalidated_in_the_same_revision(self):
        planner = self.planner()
        self.assert_certified(planner.plan(q(0), q(2)))
        path = planner.plan(q(.1), q(2))
        self.assert_certified(path)
        np.testing.assert_array_equal(path[0], q(.1))
        self.assertIn(edge_key(q(.1), q(0)), self.checker.certified_edges)

    def test_random_graph_costs_match_independent_dijkstra_with_blocked_edges(self):
        rng = np.random.default_rng(18)
        for _ in range(30):
            self.checker = Checker()
            configurations = np.array([q(i * .5, rng.uniform(-1, 1)) for i in range(9)])
            edges = np.array([(a, b) for a in range(9) for b in range(a + 1, 9)
                              if b == a + 1 or rng.random() < .25], dtype=np.int64)
            adjacency = [[] for _ in range(9)]
            for a, b in edges:
                key = edge_key(configurations[a], configurations[b])
                if rng.random() < .3:
                    self.checker.blocked_edges.add(key)
                else:
                    cost = float(np.linalg.norm(configurations[a] - configurations[b]))
                    adjacency[a].append((b, cost))
                    adjacency[b].append((a, cost))
            queue, distances = [(0., 0)], {0: 0.}
            while queue:
                cost, node = heapq.heappop(queue)
                if cost != distances[node]:
                    continue
                for neighbor, weight in adjacency[node]:
                    new_cost = cost + weight
                    if new_cost < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_cost
                        heapq.heappush(queue, (new_cost, neighbor))
            planner = self.planner(configurations, edges)
            path = planner.plan(configurations[0], configurations[-1])
            if 8 not in distances:
                self.assertIsNone(path)
            else:
                self.assert_certified(path)
                self.assertAlmostEqual(float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                                       distances[8], places=9)


if __name__ == '__main__':
    unittest.main()
