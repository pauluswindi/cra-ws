"""Resolve dataset defaults from YAML without launching ROS processes."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import yaml


ROOT = Path(__file__).resolve().parents[3]


def declarations(name, config):
    with tempfile.TemporaryDirectory(prefix='cra-launch-paths-') as temporary:
        share = Path(temporary)
        (share / 'config').mkdir()
        (share / 'config/robot.yaml').write_text(yaml.safe_dump(config))
        source = ROOT / 'src/cra/launch' / (name + '.launch.py')
        spec = importlib.util.spec_from_file_location('cra_paths_' + name, source)
        module = importlib.util.module_from_spec(spec)
        with patch('ament_index_python.packages.get_package_share_directory',
                   return_value=str(share)):
            spec.loader.exec_module(module)
            description = module.generate_launch_description()
        return {a.name: a for a in description.entities if isinstance(a, DeclareLaunchArgument)}


def default(argument):
    return perform_substitutions(LaunchContext(), argument.default_value)


class LaunchDataPathTests(unittest.TestCase):
    def test_all_launches_share_configured_yaml_workspace(self):
        config = dict(workspace=dict(path='/tmp/custom-raw.npz'),
                      gng=dict(graph_path='/tmp/custom-graph.npz'))
        for name in ('cra', 'planning', 'simulation'):
            with self.subTest(launch=name):
                args = declarations(name, config)
                self.assertEqual(default(args['workspace_path']), '/tmp/custom-raw.npz')
                if name != 'simulation':
                    self.assertEqual(default(args['graph_path']), '/tmp/custom-graph.npz')

    def test_missing_fields_preserve_previous_defaults(self):
        directory = Path.home() / 'living-lab/cra_ws/src/planning/data'
        for name in ('cra', 'planning', 'simulation'):
            with self.subTest(launch=name):
                args = declarations(name, {})
                self.assertEqual(default(args['workspace_path']),
                                 str(directory / 'raw_workspace.npz'))
                if name != 'simulation':
                    self.assertEqual(default(args['graph_path']),
                                     str(directory / 'workspace_graph.npz'))

    def test_tilde_expands_before_passing_to_planning_and_gui(self):
        config = dict(workspace=dict(path='~/raw.npz'), gng=dict(graph_path='~/graph.npz'))
        for name in ('cra', 'planning', 'simulation'):
            with self.subTest(launch=name):
                args = declarations(name, config)
                self.assertEqual(default(args['workspace_path']), str(Path.home() / 'raw.npz'))
                if name != 'simulation':
                    self.assertEqual(default(args['graph_path']), str(Path.home() / 'graph.npz'))

    def test_explicit_cli_overrides_remain_available(self):
        for name in ('cra', 'planning', 'simulation'):
            args = declarations(name, dict(workspace=dict(path='/tmp/from-yaml.npz')))
            context = LaunchContext()
            context.launch_configurations['workspace_path'] = '/tmp/from-cli.npz'
            args['workspace_path'].execute(context)
            self.assertEqual(context.launch_configurations['workspace_path'], '/tmp/from-cli.npz')

    def test_shipped_yaml_defaults_exist_without_cli_paths(self):
        config = yaml.safe_load((ROOT / 'src/cra/config/robot.yaml').read_text())
        args = declarations('cra', config)
        for name in ('workspace_path', 'graph_path'):
            self.assertTrue(Path(default(args[name])).is_file(), name)


if __name__ == '__main__':
    unittest.main()
