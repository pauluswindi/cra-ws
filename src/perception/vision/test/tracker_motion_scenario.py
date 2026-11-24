"""Event-driven, simulation-only tracker trial control and evidence evaluation.

All times are observer receipt times. A passing result covers the recorded
STOP/release/status sequence, not physical braking distance or pose accuracy.
"""


class MotionScenario:
    def __init__(self, scenario, warmup, loss_seconds=3.):
        self.scenario = scenario
        self.warmup = warmup
        self.loss_seconds = loss_seconds
        self.stage = 'prepare'
        self.motion = None
        self.geometry = None
        self.geometry_time = -1e9
        self.fault_started = None
        self.cues = []

    def cue(self, now, message):
        self.cues.append({'elapsed_s': now, 'message': message})
        return ('cue', message)

    def event(self, event):
        now, stream, state = (event[k] for k in ('elapsed_s', 'stream', 'state'))
        actions = []
        if stream == 'vision':
            self.geometry, self.geometry_time = state, now
        if stream == 'motion':
            self.motion = state
        if stream == 'target' and self.stage == 'target_requested':
            self.stage = 'awaiting_execution' if state == 'SET' else 'done'
        if stream == 'fault' and state == 'DEPTH_OFF':
            self.fault_started = now
        if stream == 'motion' and state == 'EXECUTING' and self.stage == 'awaiting_execution':
            self.stage = 'disturbance'
            if self.scenario == 'vision-loss':
                actions.extend([('command', 'DEPTH_OFF'), self.cue(
                    now, 'Robot EXECUTING: menahan depth ke Reconstruction sementara. '
                    'Tetap terlihat kamera; driver tetap berjalan.')])
            else:
                actions.append(self.cue(
                    now, 'Robot EXECUTING: halangi lintasan robot SIMULASI sekarang. '
                    'Jika robot sudah selesai, percobaan ini belum menguji halangan.'))
        elif (stream == 'motion' and state == 'STOPPED' and self.stage == 'disturbance'
              and self.scenario == 'blockage'):
            self.stage = 'recovering'
            actions.append(self.cue(
                now, 'Motion STOPPED: menjauh dari lintasan robot simulasi, '
                'tetap terlihat kamera, lalu tunggu pemulihan.'))
        elif (stream == 'motion' and state == 'COMPLETED'
              and self.stage in ('disturbance', 'recovering')):
            if self.scenario == 'vision-loss' and self.stage == 'disturbance':
                actions.append(('command', 'DEPTH_ON'))
            self.stage = 'done'
        return actions

    def tick(self, now):
        if (self.stage == 'prepare' and now >= self.warmup
                and self.geometry == 'observed' and now - self.geometry_time <= .5
                and self.motion in ('IDLE', 'COMPLETED')):
            self.stage = 'target_requested'
            return [('command', 'TARGET'), self.cue(
                now, 'Data manusia siap. Mengirim target uji dan menunggu Motion EXECUTING.')]
        if (self.scenario == 'vision-loss' and self.stage == 'disturbance'
                and self.fault_started is not None
                and now - self.fault_started >= self.loss_seconds):
            self.stage = 'recovering'
            return [('command', 'DEPTH_ON'), self.cue(
                now, 'Penerusan depth dipulihkan. Tetap terlihat kamera dan tunggu robot.')]
        return []


def evaluate_scenario(events, scenario, end_s):
    """Require evidence of activation, STOP, fresh recovery and completion.

    Cross-topic receipt order within 0.2 s is ambiguous. A command at an
    invalid/valid boundary needs review rather than a passing safety inference.
    That allowance changes only report inference, not any runtime timeout.
    """
    def first(stream, states, after=-1., before=float('inf')):
        return next((e['elapsed_s'] for e in events
                     if e['stream'] == stream and e['state'] in states
                     and after <= e['elapsed_s'] <= before), None)

    times = {}

    def result(status, reason):
        return dict(scenario=scenario, status=status, reason=reason, evidence_s=times,
                    scope='Observed command/status sequence; not physical stop distance',
                    receipt_order_uncertainty_s=.2)

    target = first('target', {'SET'})
    if target is None:
        return result('not_exercised', 'Target was not sent with fresh observed human geometry')
    times['target'] = target
    executing = first('motion', {'EXECUTING'}, target)
    if executing is None:
        return result('not_exercised', 'No execution for the requested test target')
    times['executing'] = executing
    completed = first('motion', {'COMPLETED'}, executing)
    if scenario == 'vision-loss':
        off = first('fault', {'DEPTH_OFF'}, executing)
        if off is None:
            return result('not_exercised', 'Depth withholding was not activated')
        times['depth_off'] = off
        trigger = first('vision', {'invalid'}, off)
    else:
        trigger = first('planning', {'HUMAN_TOO_CLOSE', 'HUMAN_PATH_BLOCKED'}, executing)
    if trigger is None or (completed is not None and completed < trigger):
        return result('not_exercised', 'No relevant disturbance was observed during execution')
    times['disturbance'] = trigger
    lower = times.get('depth_off', max(executing, trigger - .2))
    stop = first('command', {'STOP'}, lower)
    stopped = first('motion', {'STOPPED'}, lower)
    if stop is None or stopped is None:
        if end_s - trigger < 3.:
            return result('not_exercised', 'Insufficient observation time after disturbance')
        return result('failed', 'Missing STOP command or STOPPED status after disturbance')
    times.update(stop_command=stop, stopped=stopped)
    if scenario == 'vision-loss':
        restored = first('fault', {'DEPTH_ON'}, times['depth_off'])
        if restored is None:
            return result('failed', 'Depth forwarding was not restored')
        times['depth_on'] = restored
        ready = first('vision', {'observed'}, restored)
        # Inspect complete intervals so a delayed observer callback cannot turn
        # an ambiguous recovery boundary into a definite unsafe-command finding.
        intervals = []
        invalid_start = None
        for event in events:
            at = event['elapsed_s']
            if at < trigger:
                continue
            if event['stream'] == 'vision':
                if event['state'] == 'invalid' and invalid_start is None:
                    invalid_start = at
                elif event['state'] != 'invalid' and invalid_start is not None:
                    intervals.append((invalid_start, at))
                    invalid_start = None
        if invalid_start is not None:
            intervals.append((invalid_start, end_s))
        ambiguous = None
        for event in events:
            at = event['elapsed_s']
            if not ((event['stream'] == 'command' and event['state'] == 'RELEASE')
                    or event['stream'] == 'trajectory'
                    or (event['stream'] == 'motion' and event['state'] == 'EXECUTING')):
                continue
            for start, end in intervals:
                if start <= at <= end:
                    if start + .2 < at < end - .2:
                        times['command_during_invalid'] = at
                        return result(
                            'failed', 'Command or execution during sustained invalid data')
                    ambiguous = at
        if ambiguous is not None:
            times['ambiguous_command'] = ambiguous
            return result('not_exercised', 'Command near invalid/valid DDS boundary needs review')
    else:
        ready = first('vision', {'observed'}, stopped)
    if ready is None:
        return result('failed', 'No fresh observed human geometry during recovery')
    times['recovery_geometry'] = ready
    release = first('command', {'RELEASE'}, stopped - .2)
    resume = first('motion', {'EXECUTING'}, stopped)
    trajectory_states = {e['state'] for e in events if e['stream'] == 'trajectory'}
    trajectory = first('trajectory', trajectory_states, stopped)
    planning_finish = first('planning', {'COMPLETED'},
                            resume if resume is not None else end_s)
    finishes = [e['elapsed_s'] for e in events
                if e['stream'] == 'motion' and e['state'] == 'COMPLETED'
                and resume is not None and planning_finish is not None
                and resume <= e['elapsed_s'] <= planning_finish + .2]
    if release is None or resume is None or trajectory is None or not finishes:
        return result('failed', 'STOP observed but release, new trajectory or completion missing')
    if scenario == 'vision-loss' and resume < ready - .2:
        return result('failed', 'Execution resumed before fresh observed recovery')
    times.update(release=release, resumed=resume, trajectory=trajectory,
                 completed=finishes[-1], planning_completed=planning_finish)
    return result('passed', 'Disturbance, STOP, fresh recovery and completion were observed')
