"""Check tracker option forwarding and integer ROS parameters without devices."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

from test_launch_data_paths import declarations, default


ROOT = Path(__file__).resolve().parents[3]
LAUNCHES = (
    ROOT / 'src/cra/launch/cra.launch.py',
    ROOT / 'src/cra/launch/vision.launch.py',
    ROOT / 'src/perception/vision/launch/body_tracking.launch.py',
)


def load(path):
    spec = importlib.util.spec_from_file_location('tracker_launch_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


class TrackerLaunchTests(unittest.TestCase):
    def test_missing_yaml_settings_keep_blocking_fallback_and_accept_five_ms(self):
        for path in LAUNCHES:
            with self.subTest(path=path):
                if path == LAUNCHES[2]:
                    argument = next(a for a in load(path).entities
                                    if isinstance(a, DeclareLaunchArgument)
                                    and a.name == 'tracker_enqueue_wait_ms')
                else:
                    argument = declarations(path.name.removesuffix('.launch.py'), {})[
                        'tracker_enqueue_wait_ms']
                value = perform_substitutions(LaunchContext(), argument.default_value)
                self.assertEqual(value, '-1')
                for value in ('-1', '5'):
                    context = LaunchContext()
                    context.launch_configurations['tracker_enqueue_wait_ms'] = value
                    argument.execute(context)
                context = LaunchContext()
                context.launch_configurations['tracker_enqueue_wait_ms'] = '0'
                with self.assertRaises(RuntimeError):
                    argument.execute(context)

    def test_yaml_defaults_and_cli_overrides(self):
        for enabled, wait in ((True, 5), (False, -1)):
            config = dict(rviz=dict(enabled=enabled), vision=dict(tracker_enqueue_wait_ms=wait))
            for name in ('cra', 'vision'):
                args = declarations(name, config)
                self.assertEqual(default(args['tracker_enqueue_wait_ms']), str(wait))
                if name == 'cra':
                    self.assertEqual(default(args['rviz']), str(enabled).lower())
                    context = LaunchContext()
                    context.launch_configurations['rviz'] = str(not enabled).lower()
                    args['rviz'].execute(context)
                    self.assertEqual(context.launch_configurations['rviz'],
                                     str(not enabled).lower())
                context = LaunchContext()
                override = str(-1 if wait == 5 else 5)
                context.launch_configurations['tracker_enqueue_wait_ms'] = override
                args['tracker_enqueue_wait_ms'].execute(context)
                self.assertNotEqual(context.launch_configurations['tracker_enqueue_wait_ms'],
                                    str(wait))

    def test_yaml_rejects_unsupported_waits_and_wrong_types(self):
        for wait in (True, '5', 0, 10, 5.5):
            for name in ('cra', 'vision'):
                with self.subTest(wait=wait, launch=name):
                    with self.assertRaisesRegex(ValueError, 'vision.tracker_enqueue_wait_ms'):
                        declarations(name, dict(vision=dict(tracker_enqueue_wait_ms=wait)))
        with self.assertRaisesRegex(ValueError, 'rviz.enabled'):
            declarations('cra', dict(rviz=dict(enabled='true')))

    def test_parent_launches_forward_explicit_value(self):
        for path in LAUNCHES[:2]:
            with self.subTest(path=path):
                includes = [a for a in load(path).entities
                            if isinstance(a, IncludeLaunchDescription)]
                forwarded = [dict(a.launch_arguments)['tracker_enqueue_wait_ms'] for a in includes
                             if 'tracker_enqueue_wait_ms' in dict(a.launch_arguments)]
                self.assertEqual(len(forwarded), 1)
                context = LaunchContext()
                context.launch_configurations['tracker_enqueue_wait_ms'] = '5'
                self.assertEqual(forwarded[0].perform(context), '5')

    def test_driver_receives_integer_without_modifying_camera_settings(self):
        for value in ('-1', '5'):
            # ParameterValue caches its evaluation within one launch instance.
            with patch('launch_ros.actions.Node', wraps=Node) as factory:
                load(LAUNCHES[2])
            call = next(c for c in factory.call_args_list
                        if c.kwargs.get('package') == 'azure_kinect_ros_driver')
            camera, options = call.kwargs['parameters']
            self.assertEqual(camera['fps'], 30)
            self.assertEqual(camera['color_resolution'], '1536P')
            self.assertEqual(camera['depth_mode'], 'WFOV_2X2BINNED')
            self.assertEqual(camera['body_tracking_smoothing_factor'], .6)
            self.assertNotIn('tracker_enqueue_wait_ms', camera)
            context = LaunchContext()
            context.launch_configurations['tracker_enqueue_wait_ms'] = value
            self.assertEqual(options['tracker_enqueue_wait_ms'].evaluate(context), int(value))


if __name__ == '__main__':
    unittest.main()
