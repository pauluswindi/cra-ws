"""Run normal MuJoCo physics on the tracker comparison domain."""

from contextlib import nullcontext
import os
from pathlib import Path
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
import yaml

from simulation.robot_simulation import main


class HeadlessViewer:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def is_running(self):
        return True

    def lock(self):
        return nullcontext()

    def sync(self):
        pass


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '221' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1')
    config = Path(get_package_share_directory('cra')) / 'config/robot.yaml'
    if yaml.safe_load(config.read_text())['robot']['mode'] != 'simulation':
        raise SystemExit('Set installed robot.mode to simulation before running this trial')
    with patch('mujoco.viewer.launch_passive', return_value=HeadlessViewer()):
        main()
