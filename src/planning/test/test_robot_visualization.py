"""Numerical robot-marker checks with real MuJoCo and mocked ROS transport."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from simulation.frame_transform import FrameTransform


ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / 'src/cra/models/robot_model/cognitive_robotics_arm.xml'


class Point:
    def __init__(self, x=0., y=0., z=0.):
        self.x, self.y, self.z = x, y, z


class Marker:
    SPHERE, LINE_STRIP, LINE_LIST, POINTS, MESH_RESOURCE = 2, 4, 5, 8, 10
    ADD, DELETE = 0, 2

    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)
        self.pose = SimpleNamespace(
            position=Point(), orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
        self.scale = Point()
        self.color = SimpleNamespace(r=0., g=0., b=0., a=0.)
        self.points = []


class MarkerArray:
    def __init__(self):
        self.markers = []


class QoSProfile:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


def load_visualizer():
    replacements = {}
    for name, values in {
        'geometry_msgs.msg': {'Point': Point},
        'sensor_msgs.msg': {'JointState': SimpleNamespace},
        'visualization_msgs.msg': {'Marker': Marker, 'MarkerArray': MarkerArray},
        'rclpy.qos': {'QoSProfile': QoSProfile,
                      'DurabilityPolicy': SimpleNamespace(TRANSIENT_LOCAL=1),
                      'ReliabilityPolicy': SimpleNamespace(RELIABLE=1)},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        replacements[name] = module
        replacements.setdefault(name.split('.')[0], ModuleType(name.split('.')[0]))
    path = ROOT / 'src/planning/planning/visualization/planning_visualizer.py'
    spec = importlib.util.spec_from_file_location('cra_marker_test_module', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, replacements):
        spec.loader.exec_module(module)
    return module.PlanningVisualizer


class Node:
    def __init__(self):
        self.nanoseconds = 1_000_000_000
        self.messages = {}

    def create_publisher(self, kind, topic, qos):
        self.messages[topic] = []
        return SimpleNamespace(publish=self.messages[topic].append)

    def create_subscription(self, *args):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=self.nanoseconds,
            to_msg=lambda: SimpleNamespace(sec=1, nanosec=0)))

    def get_logger(self):
        return SimpleNamespace(info=lambda *args: None)


class RobotVisualizationTests(unittest.TestCase):
    def setUp(self):
        cls = load_visualizer()
        self.node = Node()
        with patch.object(cls, 'publish_static'):
            self.visualizer = cls(
                self.node, ROOT / 'src/planning/data/raw_workspace.npz',
                ROOT / 'src/planning/data/workspace_graph.npz', MODEL,
                [f'joint{i}' for i in range(1, 6)], 'base_link', 'end_effector')

    @staticmethod
    def rotation(quaternion):
        output = np.empty(9)
        mujoco.mju_quat2Mat(output, np.asarray(quaternion, dtype=float))
        return output.reshape(3, 3)

    def test_original_stl_poses_match_xml_and_cra_transform(self):
        visualizer = self.visualizer
        sources = {node.get('name'): node for node in ET.parse(MODEL).findall('.//geom[@mesh]')}
        assets = {node.get('name'): node for node in ET.parse(MODEL).findall('./asset/mesh')}
        configurations = [np.zeros(5), np.array([1.08, -.71, 1.24, -.16, .32])]
        configurations.append(np.random.default_rng(4).uniform(
            visualizer.model.jnt_range[:, 0], visualizer.model.jnt_range[:, 1]))
        for joints in configurations:
            visualizer.forward_kinematics(joints)
            visualizer.publish_robot()
            markers = self.node.messages['/planning/robot_model'][-1].markers
            self.assertEqual(len(markers), 6)
            base = visualizer.data.xpos[visualizer.base_id]
            conversion = (FrameTransform.ROBOT_FROM_MUJOCO_BASE
                          @ visualizer.data.xmat[visualizer.base_id].reshape(3, 3).T)
            for marker in markers:
                geom = marker.id
                source = sources[visualizer.model.geom(geom).name]
                body = int(visualizer.model.geom_bodyid[geom])
                body_rotation = visualizer.data.xmat[body].reshape(3, 3)
                source_rotation = self.rotation(
                    np.fromstring(source.get('quat', '1 0 0 0'), sep=' '))
                source_position = np.fromstring(source.get('pos', '0 0 0'), sep=' ')
                world = visualizer.data.xpos[body] + body_rotation @ source_position
                actual = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
                np.testing.assert_allclose(actual, conversion @ (world - base), atol=1e-12)
                quaternion = marker.pose.orientation
                actual_rotation = self.rotation(
                    [quaternion.w, quaternion.x, quaternion.y, quaternion.z])
                np.testing.assert_allclose(
                    actual_rotation, conversion @ body_rotation @ source_rotation, atol=1e-12)
                asset = assets[source.get('mesh')]
                np.testing.assert_allclose([marker.scale.x, marker.scale.y, marker.scale.z],
                                           np.fromstring(asset.get('scale', '1 1 1'), sep=' '))
                resource = Path(unquote(urlparse(marker.mesh_resource).path))
                self.assertEqual(resource, (MODEL.parent / asset.get('file')).resolve())
                self.assertTrue(resource.is_file())
                self.assertEqual(marker.header.frame_id, 'base_link')

    def test_bounds_match_the_five_arm_collision_geometries(self):
        visualizer = self.visualizer
        visualizer.forward_kinematics([.7, -1.1, .8, -.3, .2])
        visualizer.publish_robot()
        bounds = self.node.messages['/planning/robot_collision_bounds'][-1].markers
        self.assertEqual(len(bounds), 5)
        base = visualizer.data.xpos[visualizer.base_id]
        conversion = (FrameTransform.ROBOT_FROM_MUJOCO_BASE
                      @ visualizer.data.xmat[visualizer.base_id].reshape(3, 3).T)
        for marker in bounds:
            actual = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            expected = conversion @ (visualizer.data.geom_xpos[marker.id] - base)
            np.testing.assert_allclose(actual, expected, atol=1e-12)
            diameter = 2 * visualizer.model.geom_rbound[marker.id]
            np.testing.assert_allclose([marker.scale.x, marker.scale.y, marker.scale.z], diameter)

    def test_measured_joint_order_throttle_and_invalid_feedback(self):
        visualizer = self.visualizer
        joints = np.array([1.08, -.71, 1.24, -.16, .32])
        message = SimpleNamespace(name=list(reversed(visualizer.joint_names)),
                                  position=joints[::-1].tolist())
        # A preceding path preview can leave the shared visual model at a goal.
        visualizer.forward_kinematics([.2, -1.2, .8, -.4, .2])
        visualizer.joint_state_callback(message)
        np.testing.assert_allclose(visualizer.data.qpos[visualizer.qpos_ids], joints)
        topic = '/planning/robot_model'
        self.assertEqual(len(self.node.messages[topic]), 1)
        self.node.nanoseconds += 1_000_000
        visualizer.joint_state_callback(message)
        self.assertEqual(len(self.node.messages[topic]), 1)
        self.node.nanoseconds += 100_000_000
        visualizer.joint_state_callback(message)
        self.assertEqual(len(self.node.messages[topic]), 2)
        self.node.nanoseconds = 0
        visualizer.joint_state_callback(message)
        self.assertEqual(len(self.node.messages[topic]), 3)
        message.position[0] = np.nan
        visualizer.joint_state_callback(message)
        message.position = [0.]
        visualizer.joint_state_callback(message)
        self.assertEqual(len(self.node.messages[topic]), 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
