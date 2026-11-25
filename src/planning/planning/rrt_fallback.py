"""Bounded simulation fallback coordinator; commands belong to PlanningManager.

Reuse the shadow worker/model and real path preparation. Search output is a
candidate only; a second job certifies it on a fresh snapshot before delivery.
"""

import time

import numpy as np

from planning.rrt_shadow import ShadowRRTObserver, capture_scene, scene_expiry


def rrt_settings(planning_config, robot_mode, override=''):
    """Resolve modes before node/model creation, retaining legacy shadow YAML."""
    settings = dict(planning_config.get('rrt', planning_config.get('rrt_shadow', {})))
    enabled = settings.pop('enabled', False)
    visible = settings.pop('visualization_enabled', True)
    if not isinstance(enabled, bool) or not isinstance(visible, bool):
        raise ValueError('RRT enabled and visualization_enabled must be boolean')
    configured = settings.pop('mode', 'shadow' if enabled else 'off')
    mode = override or configured
    if mode not in ('off', 'shadow', 'fallback'):
        raise ValueError('planning.rrt.mode must be off, shadow or fallback')
    budget = settings.pop('fallback_budget', 1.5)
    if mode == 'fallback':
        if robot_mode != 'simulation':
            raise ValueError('RRT fallback is simulation-only')
        settings['fallback_budget'] = budget
    return mode, visible, settings


def fallback_scene(manager):
    checker = manager.planner.collision_filter
    scene = capture_scene(checker)
    if manager.human_status == 'OBSERVED' and checker.human_data_ready():
        scene['human_data_kind'] = 'OBSERVED'
        scene['human_valid_until'] = scene_expiry(checker, scene)
    elif (manager.human_status == 'EMPTY' and manager.human_empty_until is not None
          and time.monotonic() < manager.human_empty_until and not scene['human_capsules']):
        scene.update(human_data_kind='EMPTY', human_obstacles_enabled=False,
                     human_valid_until=manager.human_empty_until)
    else:
        raise ValueError('No fresh observed or EMPTY human snapshot')
    return scene


class RRTFallbackCoordinator(ShadowRRTObserver):
    """One job, one result mailbox, one total deadline and no Motion publishers."""

    def __init__(self, *args, fallback_budget=1.2, **kwargs):
        if (isinstance(fallback_budget, bool) or not np.isfinite(fallback_budget)
                or fallback_budget <= 0.):
            raise ValueError('fallback_budget must be positive and finite')
        super().__init__(*args, **kwargs)
        self.fallback_budget = float(fallback_budget)
        self.delivery = None
        self.deadline = None

    @staticmethod
    def context_reason(manager, attempt, start):
        request = manager.rrt_fallback_attempt
        if (not manager.rrt_fallback_pending or request is None
                or request['sequence'] != attempt['sequence'] or manager.planning_required):
            return 'PLANNING_ATTEMPT_SUPERSEDED'
        if (manager.target_base is None
                or manager.target_generation != attempt['target_generation']):
            return 'TARGET_CHANGED'
        if (manager.fault or manager.motion_active or manager.pending_action is not None
                or manager.motion_status not in ('IDLE', 'COMPLETED', 'STOPPED')
                or manager.recovery_pending or manager.resume_after_fault
                or manager.safe_recovery_active or manager.safe_position_active
                or manager.human_margin_recovery_active
                or (manager.motion_status == 'STOPPED' and not manager.stop_owned)):
            return 'MOTION_CONTEXT_CHANGED'
        if not manager.feedback_ready():
            return 'FEEDBACK_STALE'
        if not manager.obstacle_data_ready():
            return 'OBSTACLES_STALE'
        if np.max(np.abs(manager.current_q - start)) > .01:
            return 'START_MOVED'
        if not manager.rrt_human_ready():
            return 'HUMAN_VISION_UNKNOWN'
        return None

    def _failure(self, reason):
        self.delivery = dict(sequence=self.attempt['sequence'],
                             target_generation=self.attempt['target_generation'], reason=reason)
        return dict(event='FALLBACK_REJECTED', mode='fallback', observation_only=False,
                    reason=reason, attempt=self.attempt['sequence'])

    def tick(self, manager):
        if self.disabled:
            if manager.rrt_fallback_pending:
                self.attempt = dict(manager.rrt_fallback_attempt)
                return self._failure('WORKER_DISABLED')
            return None
        try:
            return self._poll(manager)
        except Exception as error:
            self.disabled = True
            if self.attempt is None:
                self.attempt = dict(manager.rrt_fallback_attempt)
            event = self._failure('WORKER_ERROR')
            event['error'] = f'{type(error).__name__}: {error}'
            return event

    def _poll(self, manager):
        now = time.monotonic()
        if self.future is not None:
            reason = self.context_reason(manager, self.attempt, self.attempt['start'])
            if self.discard_reason is None:
                self.discard_reason = reason
                if reason is not None:
                    # Report failure promptly, but drain the running job before
                    # another submission. A late result can never replace it.
                    return self._failure(reason)
            if self.discard_reason is None and now >= self.deadline:
                self.discard_reason = 'TIME_BUDGET_EXHAUSTED'
                return self._failure(self.discard_reason)
            if not self.future.done():
                return None
            result = self.future.result()
            self.future = None
            if self.discard_reason is not None:
                return dict(event='DISCARDED', mode='fallback', observation_only=False,
                            disposition=self.discard_reason, attempt=self.attempt['sequence'])
            visualization = result.pop('visualization', None)
            if visualization is not None:
                self.visualization_result = dict(
                    visualization, mode='fallback',
                    target_generation=self.attempt['target_generation'])
            event = dict(event='RESULT', mode='fallback', observation_only=False,
                         attempt=self.attempt['sequence'], kind=result['kind'],
                         reason=result['reason'], worker_pid=result.get('worker_pid'),
                         elapsed_s=result.get('elapsed_s'),
                         capture_to_result_s=now - self.job['captured_at'])
            if result['reason'] != 'NONE':
                return dict(event, **self._failure(result['reason']))
            if result['kind'] == 'search':
                path = np.asarray(result['path'], dtype=float)
                if (path.ndim != 2 or path.shape[1] != 5 or not 2 <= len(path) <= 3000
                        or not np.isfinite(path).all()
                        or not np.array_equal(path[0], self.attempt['start'])
                        or not np.array_equal(path[-1], self.attempt['goal'])):
                    return self._failure('ENDPOINT_MISMATCH')
                scene = fallback_scene(manager)
                allowance = min(self.settings['validation_budget'], self.deadline - now,
                                scene_expiry(manager.planner.collision_filter, scene) - now)
                if allowance <= 0.:
                    return self._failure('TIME_BUDGET_EXHAUSTED')
                self._submit(dict(kind='validate', scene=scene, captured_at=now,
                                  start=manager.current_q.copy(), path=path,
                                  max_segment_delta=manager.planner.max_segment_delta,
                                  sample_step=self.sample_step, total_budget=allowance,
                                  request_deadline=min(self.deadline, now + allowance),
                                  return_prepared=True))
                return dict(event, event='LATEST_VALIDATION_SUBMITTED')
            checker = manager.planner.collision_filter
            changed = checker.environment_revision != self.job['scene']['revision']
            expiry = scene_expiry(checker, self.job['scene'])
            if now >= expiry or now >= self.job['request_deadline']:
                return self._failure('VALIDATION_SNAPSHOT_EXPIRED')
            if np.max(np.abs(manager.current_q - self.job['start'])) > .01:
                return self._failure('START_MOVED')
            prepared = np.asarray(result.get('prepared_path'), dtype=float)
            self.delivery = dict(
                sequence=self.attempt['sequence'],
                target_generation=self.attempt['target_generation'], reason='NONE',
                # Preserve the certificate's actual revision. A changed scene
                # makes this a candidate, never a certificate for the new scene.
                scene_revision=self.job['scene']['revision'],
                expires_at=min(expiry, self.deadline),
                prepared_path=prepared, start=self.job['start'].copy())
            return dict(event, event=('CANDIDATE_REVALIDATION_REQUIRED' if changed
                                      else 'FALLBACK_VALIDATED'), points=len(prepared))

        request = manager.rrt_fallback_attempt
        if not manager.rrt_fallback_pending or request is None:
            return None
        if request['sequence'] <= self.last_sequence:
            return None
        self.attempt = dict(request)
        reason = self.context_reason(manager, self.attempt, self.attempt['start'])
        if reason is not None:
            self.last_sequence = request['sequence']
            return self._failure(reason)
        if now >= request['deadline']:
            self.last_sequence = request['sequence']
            return self._failure('TIME_BUDGET_EXHAUSTED')
        if now - self.last_started < self.settings['retry_interval']:
            # Keep one pending request. Do not manufacture a failed search or
            # advance last_sequence until it can actually be admitted.
            return None
        self.last_sequence = request['sequence']
        if request.pop('waiting_retry', False):
            request['deadline'] = now + self.fallback_budget
        self.attempt = dict(request)
        scene = fallback_scene(manager)
        self.deadline = min(request['deadline'], now + self.fallback_budget,
                            scene_expiry(manager.planner.collision_filter, scene))
        allowance = min(self.settings['search_budget'], self.deadline - now)
        if allowance <= 0.:
            return self._failure('TIME_BUDGET_EXHAUSTED')
        self.discard_reason = None
        self.last_started = now
        manager.rrt_retry_not_before = now + self.settings['retry_interval']
        self._submit(dict(kind='search', scene=scene, captured_at=now,
                          start=self.attempt['start'].copy(), goal=self.attempt['goal'].copy(),
                          seed=request['sequence'], search_budget=allowance, total_budget=allowance,
                          sample_step=self.sample_step, request_deadline=self.deadline,
                          use_preference=False, visualize=self.visualization_requested,
                          projection_bodies=self.projection_bodies))
        return dict(event='SEARCH_SUBMITTED', mode='fallback', observation_only=False,
                    attempt=request['sequence'], scene_revision=scene['revision'])
