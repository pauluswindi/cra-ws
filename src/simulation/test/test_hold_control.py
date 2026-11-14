"""Exercise simulation hold and tracking with MuJoCo and stub ROS imports."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]


def simulation_class():
    replacements = {}
    for name, values in {
        'rclpy': {},
        'rclpy.node': {'Node': object},
        'rclpy.qos': dict.fromkeys(['DurabilityPolicy', 'QoSProfile', 'ReliabilityPolicy']),
        'ament_index_python.packages': {'get_package_share_directory': None},
        'geometry_msgs.msg': dict.fromkeys(['Point', 'PoseArray']),
        'sensor_msgs.msg': {'JointState': None},
        'std_msgs.msg': {'Float64MultiArray': None},
        'visualization_msgs.msg': dict.fromkeys(['Marker', 'MarkerArray']),
    }.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        replacements[name] = module
        replacements.setdefault(name.split('.')[0], ModuleType(name.split('.')[0]))
    path = ROOT / 'src/simulation/simulation/robot_simulation.py'
    spec = importlib.util.spec_from_file_location('cra_hold_test_module', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, replacements):
        spec.loader.exec_module(module)
    return module.RobotSimulation


class SimulationHoldTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        cls = simulation_class()
        self.robot = cls.__new__(cls)
        robot = self.robot
        robot.model = mujoco.MjModel.from_xml_path(str(
            ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml'))
        robot.data = mujoco.MjData(robot.model)
        robot.qpos_ids = list(range(5))
        robot.qvel_ids = list(range(5))
        robot.actuator_ids = list(range(5))
        robot.q_lower = robot.model.jnt_range[:, 0].copy()
        robot.q_upper = robot.model.jnt_range[:, 1].copy()
        robot.data.qpos[:] = [1.98576289, -.64575674, 1.22786207, -.91740616, -.94390645]
        mujoco.mj_forward(robot.model, robot.data)
        robot.q_des = robot.hold_position_target()
        robot.command = np.zeros(5)
        robot.max_velocity = .5
        robot.command_timeout = .2
        robot.holding_position = False
        robot.last_command = 100.
        robot.last_debug_time = 100.
        robot.pending_target = None
        robot.pending_obstacles = {}
        robot.dt = robot.model.opt.timestep
        robot.get_logger = lambda: SimpleNamespace(error=lambda *args: None)

    def assert_stationary(self):
        robot = self.robot
        initial = robot.joint_positions()
        peak = 0.
        for _ in range(round(.5 / robot.dt)):
            robot.update_control()
            mujoco.mj_step(robot.model, robot.data)
            peak = max(peak, float(np.max(np.abs(robot.joint_positions() - initial))))
        self.assertLess(peak, 1e-9)
        self.assertLess(float(np.max(np.abs(robot.data.qvel))), 1e-9)
        self.assertEqual(robot.data.ncon, 0)

    def test_zero_command_balances_gravity_instead_of_starting_a_new_motion(self):
        self.robot.velocity_callback(SimpleNamespace(data=[0.] * 5))
        self.assertTrue(self.robot.holding_position)
        self.assert_stationary()

    def test_command_timeout_uses_the_same_gravity_balanced_hold(self):
        self.now.return_value = 100.21
        self.assert_stationary()
        self.assertTrue(self.robot.holding_position)
        self.assertIsNone(self.robot.last_command)

    def test_invalid_command_uses_the_same_hold(self):
        self.robot.velocity_callback(SimpleNamespace(data=[float('nan')] * 5))
        self.assert_stationary()

    def test_repeated_zero_commands_do_not_recapture_the_hold_target(self):
        robot = self.robot
        robot.velocity_callback(SimpleNamespace(data=[0.] * 5))
        target = robot.q_des.copy()
        robot.data.qpos[0] += .01
        mujoco.mj_forward(robot.model, robot.data)
        robot.velocity_callback(SimpleNamespace(data=[0.] * 5))
        np.testing.assert_array_equal(robot.q_des, target)
        self.assertTrue(np.all(target >= robot.q_lower))
        self.assertTrue(np.all(target <= robot.q_upper))

    def assert_tracks_path(self, path):
        """Apply Motion's 200 Hz linear reference and P law without ROS."""
        robot = self.robot
        path = np.asarray(path, dtype=float)
        robot.data.qpos[:] = path[0]
        robot.data.qvel[:] = 0.
        mujoco.mj_forward(robot.model, robot.data)
        robot.q_des = robot.hold_position_target()
        robot.holding_position = True
        durations = np.max(np.abs(np.diff(path, axis=0)), axis=1) / .3
        ends = np.cumsum(durations)
        tracking_violation_time = 0.
        peak_error = 0.
        completed = False
        next_command = 0.
        for index in range(round((ends[-1] + 2.) / robot.dt)):
            elapsed = index * robot.dt
            if elapsed + 1e-12 >= next_command:
                next_command += .005
                segment = min(int(np.searchsorted(ends, elapsed, side='right')),
                              len(durations) - 1)
                begin = ends[segment - 1] if segment else 0.
                fraction = np.clip((elapsed - begin) / durations[segment], 0., 1.)
                reference = path[segment] + fraction * (path[segment + 1] - path[segment])
                feedforward = ((path[segment + 1] - path[segment]) / durations[segment]
                               if elapsed < ends[-1] else np.zeros(5))
                error = reference - robot.joint_positions()
                worst_error = float(np.max(np.abs(error)))
                peak_error = max(peak_error, worst_error)
                tracking_violation_time = (
                    tracking_violation_time + .005 if worst_error > .2 else 0.)
                self.assertLess(tracking_violation_time, .5, 'TRACKING_ERROR would stop Motion')
                if elapsed >= ends[-1] and worst_error <= .02:
                    robot.velocity_callback(SimpleNamespace(data=[0.] * 5))
                    completed = True
                    break
                robot.velocity_callback(SimpleNamespace(data=feedforward + 2. * error))
                self.assertLessEqual(float(np.max(np.abs(robot.command))), .5)
            robot.update_control()
            self.assertTrue(np.all(robot.q_des >= robot.q_lower))
            self.assertTrue(np.all(robot.q_des <= robot.q_upper))
            mujoco.mj_step(robot.model, robot.data)
            self.assertEqual(robot.data.ncon, 0)
        self.assertTrue(completed, 'Trajectory did not reach Motion goal tolerance')
        self.assertLess(peak_error, .04)
        self.assertTrue(robot.holding_position)

    def test_lifting_from_logged_fault_poses_preserves_gravity_support(self):
        poses = [
            [2.676689475, -2.01823465, 2.52959952, .2498111, .7101838],
            [2.65816111, -2.2929963, 2.18411684, .30585713, 1.02859964],
            [2.539579529, -2.278715205, 2.442993145, -.051390768, 1.279583919],
        ]
        for pose in poses:
            with self.subTest(joint2=pose[1]):
                goal = np.array(pose)
                goal[1] += .6
                self.assert_tracks_path([pose, goal])

    def test_planned_path_from_last_logged_pose_tracks_without_fault(self):
        # Actual graph waypoints for the logged target, with the long edge split
        # to respect Motion's 0.6 rad maximum segment delta.
        start = np.array([2.539579529, -2.278715205, 2.442993145, -.051390768, 1.279583919])
        middle = np.array([2.547122955, -1.494770050, 2.598955393, -.354492635, 1.277083397])
        goal = np.array([2.464641809, -1.196630239, 2.809971571, -.719650507, 1.238520741])
        self.assert_tracks_path([start, (start + middle) / 2., middle, goal])


if __name__ == '__main__':
    unittest.main()
