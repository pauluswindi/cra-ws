"""Check the built package, without importing from the source workspace."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PlanningPackageInstallationTests(unittest.TestCase):
    def test_built_package_contains_the_visualizer_imported_by_planning_node(self):
        with tempfile.TemporaryDirectory(prefix='cra-planning-package-') as temporary:
            root = Path(temporary)
            source = root / 'source'
            source.mkdir()
            for name in ('setup.py', 'setup.cfg', 'package.xml'):
                shutil.copy2(PACKAGE_ROOT / name, source / name)
            shutil.copytree(PACKAGE_ROOT / 'planning', source / 'planning',
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            built = root / 'built'
            environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
            result = subprocess.run(
                [sys.executable, 'setup.py', 'build_py', '--build-lib', str(built)],
                cwd=source, env=environment, text=True, capture_output=True,
                timeout=60, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue((built / 'planning/visualization/planning_visualizer.py').is_file(),
                            'PlanningVisualizer was omitted from the built package')
            # Isolated mode excludes source/PYTHONPATH; find_spec checks the
            # installed module location without requiring ROS to be imported.
            result = subprocess.run(
                [sys.executable, '-I', '-c',
                 'import importlib.util, pathlib, sys; '
                 'sys.path.insert(0, sys.argv[1]); '
                 'spec = importlib.util.find_spec("planning.visualization.planning_visualizer"); '
                 'assert spec is not None; '
                 'assert pathlib.Path(spec.origin).is_relative_to(sys.argv[1])', str(built)],
                cwd=root, env=environment, text=True, capture_output=True,
                timeout=30, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
