"""Native DDS display probe on domain 227; no Motion node, viewer or live camera.

Use the full stored GNG. Cancel ordinary Planning and RRT timers; verify only
XYZ masking, original-data preservation and change-only marker transport.
"""

import argparse
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import time
from unittest.mock import patch

from geometry_msgs.msg import Pose, PoseArray
from msgs.msg import HumanCapsule, HumanCapsuleArray
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray

from benchmark_rrt_connect import CONFIG, GRAPH, MODEL, ROOT, START
from planning.planning_node import PlanningNode
from simulation.frame_transform import FrameTransform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '227':
        parser.error('Use isolated ROS_DOMAIN_ID=227')
    rclpy.init(args=['--ros-args', '-p', f'graph_path:={GRAPH}',
                     '-p', f'model_path:={MODEL}', '-p', f'config_path:={CONFIG}',
                     '-p', f'workspace_path:={ROOT / "src/planning/data/raw_workspace.npz"}'])
    node = feeder = executor = None
    try:
        node = PlanningNode()
        node.timer.cancel()
        if node.rrt_shadow is not None:
            node.rrt_shadow_timer.cancel()
        assert not hasattr(node, 'filtered_graph_display')
        feeder = Node('workspace_mask_synthetic_probe', use_global_arguments=False)
        joint_pub = feeder.create_publisher(JointState, '/joint_states', 10)
        human_pub = feeder.create_publisher(HumanCapsuleArray, '/vision/human/geometry', 10)
        obstacle_pub = feeder.create_publisher(PoseArray, '/simulation/obstacle_poses', 10)
        arrays, commands = [], []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        handles = [
            feeder.create_subscription(MarkerArray, '/planning/workspace_graph_mask',
                                       arrays.append, qos),
            feeder.create_subscription(JointTrajectory, '/motion/joint_path',
                                       lambda _: commands.append('EXECUTE'), 10),
            feeder.create_subscription(Bool, '/motion/stop_requested',
                                       lambda _: commands.append('STOP_OR_RELEASE'), 10),
        ]
        checker = node.planner.collision_filter
        original = node.visualizer.gng_nodes.copy(), node.visualizer.gng_edges.copy()
        center = original[0][400]
        human_center = [10., 10., 10.]
        human_valid, human_empty, send_human = True, False, False
        override = {}

        def publish():
            stamp = feeder.get_clock().now().to_msg()
            joint = JointState()
            joint.header.stamp = stamp
            joint.name, joint.position = node.joint_names, START.tolist()
            joint_pub.publish(joint)
            if send_human:
                human = HumanCapsuleArray()
                human.header.stamp, human.header.frame_id = stamp, node.base_frame
                human.valid = human_valid
                if not human_empty:
                    capsule = HumanCapsule()
                    capsule.body_id = capsule.segment_id = 0
                    capsule.start.x, capsule.start.y, capsule.start.z = (
                        float(v) for v in human_center)
                    capsule.end.x, capsule.end.y, capsule.end.z = (
                        float(v) for v in np.asarray(human_center) + [0., 0., .1])
                    capsule.radius = .05
                    human.capsules = [capsule]
                human_pub.publish(human)
            obstacles = PoseArray()
            obstacles.header.frame_id = 'mujoco_world'
            for name in node.obstacle_names:
                mocap = checker.obstacle_geoms[name]['mocap_id']
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    float(v) for v in override.get(name, checker.data.mocap_pos[mocap]))
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
                    float(v) for v in checker.data.mocap_quat[mocap])
                obstacles.poses.append(pose)
            obstacle_pub.publish(obstacles)
            node.monitor_humans()
            # Guard the display call specifically; normal feedback/geometry
            # callbacks outside it retain their real checker behavior.
            with (patch.object(checker, 'is_state_valid',
                               side_effect=AssertionError('state query')),
                  patch.object(checker, 'is_edge_valid', side_effect=AssertionError('edge query'))):
                node.update_graph_visualization()

        timer = feeder.create_timer(.05, publish)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(feeder)

        def latest():
            if not arrays:
                return None
            markers = arrays[-1].markers
            if len(markers) != 3 or markers[0].action != markers[0].ADD:
                return None
            return dict(nodes=len(markers[0].points), edges=len(markers[1].points) // 2,
                        node_alpha=markers[0].color.a, edge_alpha=markers[1].color.a,
                        status=markers[2].text)

        def wait_for(predicate, timeout=5.):
            began = time.monotonic()
            while time.monotonic() - began < timeout:
                executor.spin_once(timeout_sec=.01)
                current = latest()
                if current is not None and predicate(current):
                    return current
            raise AssertionError(latest())

        stages = []
        absent = wait_for(lambda x: 'UNKNOWN' in x['status']
                          and 'STALE' not in x['status'])
        assert absent['nodes'] > 0 and absent['node_alpha'] > .8, absent
        stages.append(dict(stage='startup_without_camera', **absent))
        send_human = True
        far = wait_for(lambda x: 'OBSERVED' in x['status'])
        stages.append(dict(stage='far_human', **far))
        before = len(arrays), node.workspace_graph_mask.calculations
        until = time.monotonic() + .8
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        stable = dict(extra_marker_arrays=len(arrays) - before[0],
                      extra_mask_calculations=node.workspace_graph_mask.calculations - before[1])
        assert stable == dict(extra_marker_arrays=0, extra_mask_calculations=0), stable
        human_center = center
        blocked = wait_for(lambda x: x['nodes'] < far['nodes'] and 'OBSERVED' in x['status'])
        stages.append(dict(stage='blocking_human', **blocked))
        human_center = [10., 10., 10.]
        away = wait_for(lambda x: x['nodes'] == far['nodes'] and x['edges'] == far['edges'])
        stages.append(dict(stage='human_away', **away))
        base = checker.data.xpos[checker.base_id]
        rotation = checker.data.xmat[checker.base_id].reshape(3, 3)
        override['obstacle_1'] = base + rotation @ FrameTransform.robot_base_to_mujoco_base(center)
        box = wait_for(lambda x: x['nodes'] < far['nodes'])
        stages.append(dict(stage='blocking_simulation_box', **box))
        override['obstacle_1'] = np.full(3, 100.)
        wait_for(lambda x: x['nodes'] == far['nodes'] and x['edges'] == far['edges'])
        human_empty = True
        empty = wait_for(lambda x: 'EMPTY' in x['status'])
        stages.append(dict(stage='fresh_empty_human', **empty))
        human_valid = False
        unknown = wait_for(lambda x: 'UNKNOWN' in x['status'])
        assert unknown['nodes'] == far['nodes'] and unknown['edges'] == far['edges']
        assert unknown['node_alpha'] == far['node_alpha']
        stages.append(dict(stage='unknown_human_boxes_still_masked', **unknown))
        override['obstacle_1'] = base + rotation @ FrameTransform.robot_base_to_mujoco_base(center)
        box_without_camera = wait_for(lambda x: x['nodes'] < far['nodes']
                                      and 'UNKNOWN' in x['status'])
        stages.append(dict(stage='moving_box_without_camera', **box_without_camera))
        override['obstacle_1'] = np.full(3, 100.)
        wait_for(lambda x: x['nodes'] == far['nodes'] and x['edges'] == far['edges'])
        human_valid, human_empty = True, False
        wait_for(lambda x: 'OBSERVED' in x['status'])
        send_human = False
        expired = wait_for(lambda x: 'UNKNOWN' in x['status'], timeout=4.)
        assert expired['nodes'] == far['nodes'] and expired['node_alpha'] == far['node_alpha']
        stages.append(dict(stage='expired_human_graph_and_box_mask_visible', **expired))
        np.testing.assert_array_equal(original[0], node.visualizer.gng_nodes)
        np.testing.assert_array_equal(original[1], node.visualizer.gng_edges)
        assert not commands, commands
        assert not multiprocessing.active_children()
        report = dict(scope=__doc__, ros_domain=227, graph_nodes=len(original[0]),
                      graph_edges=len(original[1]), stages=stages, unchanged_scene=stable,
                      motion_commands=commands, mask_worker_processes=0,
                      old_filtered_topic_publishers=len(node.get_publishers_info_by_topic(
                          '/planning/filtered_joint_graph')),
                      normal_planning_timer_cancelled=True)
        files = [Path(__file__), CONFIG, GRAPH, ROOT / 'src/cra/config/cra.rviz',
                 ROOT / 'src/planning/planning/planning_node.py',
                 ROOT / 'src/planning/planning/visualization/planning_visualizer.py',
                 ROOT / 'src/planning/planning/visualization/workspace_graph_mask.py']
        report['source_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(
            p.read_bytes()).hexdigest() for p in files}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        assert handles and timer
    finally:
        if executor is not None:
            executor.shutdown()
        if feeder is not None:
            feeder.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
