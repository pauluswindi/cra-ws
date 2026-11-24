"""Regression checks for trial eligibility and event-driven disturbance control."""

import unittest

from tracker_motion_scenario import MotionScenario, evaluate_scenario


def event(at, stream, state):
    return dict(elapsed_s=at, stream=stream, state=state)


def loss_events():
    return [event(10., 'target', 'SET'), event(10.3, 'motion', 'EXECUTING'),
            event(10.4, 'fault', 'DEPTH_OFF'), event(10.95, 'command', 'STOP'),
            event(10.96, 'motion', 'STOPPED'), event(10.98, 'vision', 'invalid'),
            event(13.4, 'fault', 'DEPTH_ON'), event(13.5, 'vision', 'observed'),
            event(13.7, 'command', 'RELEASE'), event(13.8, 'trajectory', '2'),
            event(13.81, 'motion', 'EXECUTING'), event(16., 'motion', 'COMPLETED'),
            event(16.1, 'planning', 'COMPLETED')]


class MotionScenarioTests(unittest.TestCase):
    def test_initial_execution_does_not_trigger_disturbance(self):
        controller = MotionScenario('vision-loss', 15.)
        self.assertEqual(controller.event(event(2., 'motion', 'EXECUTING')), [])
        self.assertEqual(controller.stage, 'prepare')

    def test_fresh_geometry_and_idle_required_for_target(self):
        controller = MotionScenario('vision-loss', 15.)
        controller.event(event(14., 'vision', 'observed'))
        controller.event(event(14., 'motion', 'COMPLETED'))
        self.assertEqual(controller.tick(15.), [])
        controller.event(event(15.1, 'vision', 'observed'))
        self.assertIn(('command', 'TARGET'), controller.tick(15.2))
        self.assertEqual(controller.tick(15.3), [])

    def test_loss_waits_for_target_execution_and_native_fault_ack(self):
        controller = MotionScenario('vision-loss', 15.)
        controller.stage = 'target_requested'
        controller.event(event(15., 'target', 'SET'))
        self.assertEqual(controller.tick(16.), [])
        actions = controller.event(event(16.1, 'motion', 'EXECUTING'))
        self.assertIn(('command', 'DEPTH_OFF'), actions)
        self.assertEqual(controller.tick(20.), [])
        controller.event(event(20.1, 'fault', 'DEPTH_OFF'))
        self.assertEqual(controller.tick(23.), [])
        self.assertIn(('command', 'DEPTH_ON'), controller.tick(23.2))

    def test_blockage_recovery_cue_waits_for_stopped(self):
        controller = MotionScenario('blockage', 15.)
        controller.stage = 'awaiting_execution'
        self.assertEqual(len(controller.event(event(16., 'motion', 'EXECUTING'))), 1)
        self.assertEqual(controller.event(event(16.1, 'planning', 'HUMAN_PATH_BLOCKED')), [])
        self.assertEqual(len(controller.event(event(16.2, 'motion', 'STOPPED'))), 1)

    def test_complete_before_disturbance_restores_depth(self):
        controller = MotionScenario('vision-loss', 15.)
        controller.stage = 'disturbance'
        self.assertIn(('command', 'DEPTH_ON'),
                      controller.event(event(17., 'motion', 'COMPLETED')))
        self.assertEqual(controller.stage, 'done')

    def test_loss_passes_with_cross_topic_stop_receipt_reordering(self):
        result = evaluate_scenario(loss_events(), 'vision-loss', 40.)
        self.assertEqual(result['status'], 'passed')
        self.assertLess(result['evidence_s']['stopped'], result['evidence_s']['disturbance'])

    def test_old_timed_trial_is_not_evidence_of_avoidance(self):
        events = [event(25., 'target', 'SET'), event(25.4, 'motion', 'EXECUTING'),
                  event(27.3, 'motion', 'COMPLETED'), event(35., 'phase', '2')]
        self.assertEqual(evaluate_scenario(events, 'blockage', 55.)['status'], 'not_exercised')

    def test_loss_after_completion_is_not_exercised(self):
        events = loss_events()
        events.insert(3, event(10.6, 'motion', 'COMPLETED'))
        events.sort(key=lambda e: e['elapsed_s'])
        self.assertEqual(evaluate_scenario(events, 'vision-loss', 40.)['status'], 'not_exercised')

    def test_release_during_sustained_invalid_data_fails(self):
        events = loss_events() + [event(12., 'command', 'RELEASE')]
        events.sort(key=lambda e: e['elapsed_s'])
        self.assertEqual(evaluate_scenario(events, 'vision-loss', 40.)['status'], 'failed')

    def test_release_near_recovery_boundary_needs_review(self):
        events = loss_events() + [event(13.49, 'command', 'RELEASE')]
        events.sort(key=lambda e: e['elapsed_s'])
        result = evaluate_scenario(events, 'vision-loss', 40.)
        self.assertEqual(result['status'], 'not_exercised')
        self.assertIn('boundary', result['reason'])

    def test_missing_stop_fails(self):
        events = [e for e in loss_events() if e['state'] not in ('STOP', 'STOPPED')]
        self.assertEqual(evaluate_scenario(events, 'vision-loss', 40.)['status'], 'failed')

    def test_recovery_without_new_trajectory_does_not_pass(self):
        events = [e for e in loss_events() if e['stream'] != 'trajectory']
        self.assertEqual(evaluate_scenario(events, 'vision-loss', 40.)['status'], 'failed')

    def test_intermediate_motion_completion_without_planner_completion_does_not_pass(self):
        events = [e for e in loss_events()
                  if not (e['stream'] == 'planning' and e['state'] == 'COMPLETED')]
        self.assertEqual(evaluate_scenario(events, 'vision-loss', 40.)['status'], 'failed')

    def test_blockage_requires_human_reason_then_stop_and_recovery(self):
        events = [event(10., 'target', 'SET'), event(10.3, 'motion', 'EXECUTING'),
                  event(11., 'planning', 'HUMAN_PATH_BLOCKED'),
                  event(11.01, 'command', 'STOP'), event(11.02, 'motion', 'STOPPED'),
                  event(12., 'vision', 'observed'), event(12.2, 'command', 'RELEASE'),
                  event(12.3, 'trajectory', '3'), event(12.31, 'motion', 'EXECUTING'),
                  event(15., 'motion', 'COMPLETED'), event(15.1, 'planning', 'COMPLETED')]
        self.assertEqual(evaluate_scenario(events, 'blockage', 40.)['status'], 'passed')


if __name__ == '__main__':
    unittest.main()
