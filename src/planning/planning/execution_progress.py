"""Ordered local segment evidence. A consumer must validate any proposed trim."""

import numpy as np


class OrderedSegmentProgress:
    def __init__(self, path, corridor_tolerance=.04, confirmation_samples=2,
                 handoff_fraction=.9, max_feedback_step=.05, next_waypoint=1):
        self.path = np.asarray(path, dtype=float).copy()
        self.tolerance = float(corridor_tolerance)
        self.confirmations = int(confirmation_samples)
        self.handoff_fraction = float(handoff_fraction)
        self.max_feedback_step = float(max_feedback_step)
        if (
            self.path.ndim != 2 or self.path.shape[1] != 5 or len(self.path) < 2
            or not np.isfinite(self.path).all()
            or not np.isfinite(self.tolerance) or self.tolerance <= 0.
            or self.confirmations != confirmation_samples or self.confirmations < 2
            or not np.isfinite(self.handoff_fraction) or not 0. < self.handoff_fraction < 1.
            or not np.isfinite(self.max_feedback_step) or self.max_feedback_step <= 0.
        ):
            raise ValueError('Invalid ordered segment progress configuration')
        self.next_waypoint = int(next_waypoint)
        if not 1 <= self.next_waypoint < len(self.path):
            raise ValueError('Invalid progress cursor')
        self.armed = False
        self.confirmed_samples = 0
        self.reason = 'WAITING_FOR_ORDERED_APPROACH'

    def _project(self, q, edge):
        a, b = self.path[edge:edge + 2]
        delta = b - a
        length2 = float(delta @ delta)
        if length2 <= 1e-18:
            return 0., float(np.max(np.abs(q - a)))
        fraction = float((q - a) @ delta) / length2
        closest = a + np.clip(fraction, 0., 1.) * delta
        return fraction, float(np.max(np.abs(q - closest)))

    def synchronize(self, next_waypoint):
        """Direct measured waypoint passage can supersede local segment evidence."""
        if next_waypoint > self.next_waypoint:
            self.next_waypoint = int(next_waypoint)
            self.armed = False
            self.confirmed_samples = 0

    def observe(self, previous, current):
        previous = np.asarray(previous, dtype=float)
        current = np.asarray(current, dtype=float)
        if (previous.shape != (5,) or current.shape != (5,)
                or not np.isfinite(previous).all() or not np.isfinite(current).all()):
            self.armed = False
            self.confirmed_samples = 0
            self.reason = 'INVALID_FEEDBACK'
            return
        movement = current - previous
        if np.max(np.abs(movement)) > self.max_feedback_step:
            self.armed = False
            self.confirmed_samples = 0
            self.reason = 'FEEDBACK_JUMP'
            return
        if np.max(np.abs(movement)) <= 1e-9:
            self.confirmed_samples = 0
            self.reason = 'STATIONARY'
            return
        if self.next_waypoint >= len(self.path) - 1:
            self.reason = 'FINAL_SEGMENT'
            return
        incoming = self.next_waypoint - 1
        outgoing = self.next_waypoint
        old_fraction, old_error = self._project(previous, incoming)
        fraction, error = self._project(current, incoming)
        if (
            old_error <= self.tolerance and error <= self.tolerance
            and fraction > old_fraction
            and self.handoff_fraction <= fraction <= 1.
        ):
            self.armed = True
        elif (self.armed and error <= self.tolerance
              and fraction < self.handoff_fraction / 2. and fraction < old_fraction):
            self.armed = False
            self.confirmed_samples = 0
            self.reason = 'BACKWARD_APPROACH'
            return
        new_fraction, new_error = self._project(current, outgoing)
        previous_fraction, previous_error = self._project(previous, outgoing)
        if not self.armed:
            self.confirmed_samples = 0
            self.reason = 'WAITING_FOR_ORDERED_APPROACH'
            return
        # Require a continuous transition through the local corridor union.
        if min(error, new_error) > self.tolerance:
            self.armed = False
            self.confirmed_samples = 0
            self.reason = 'OUTSIDE_LOCAL_CORRIDOR'
            return
        if (
            error > self.tolerance and new_error <= self.tolerance
            and min(old_error, previous_error) <= self.tolerance
            and 0. < new_fraction <= 1. and new_fraction > previous_fraction + 1e-9
        ):
            for position in (.25, .5, .75):
                probe = previous + position * movement
                if min(self._project(probe, incoming)[1],
                       self._project(probe, outgoing)[1]) > self.tolerance:
                    self.armed = False
                    self.confirmed_samples = 0
                    self.reason = 'FEEDBACK_CHORD_OUTSIDE_CORRIDOR'
                    return
            self.confirmed_samples += 1
            self.reason = 'CONFIRMING_NEXT_SEGMENT'
            if self.confirmed_samples >= self.confirmations:
                self.next_waypoint += 1
                self.armed = False
                self.confirmed_samples = 0
                self.reason = 'CANDIDATE_NEXT_SEGMENT'
        else:
            self.confirmed_samples = 0
            self.reason = 'WAITING_TO_LEAVE_INCOMING_CORRIDOR'
