"""Headless ROS transport probe on an isolated domain; no robot or Motion node.

The ordinary planning timer is cancelled deliberately. An artificial graph
failure exposes the new observer while real DDS joint/human callbacks run.
This does not prove whole-cycle responsiveness of the existing synchronous A*.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import time

from geometry_msgs.msg import Pose, PoseArray
from msgs.msg import HumanCapsule, HumanCapsuleArray
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory

from benchmark_rrt_connect import CONFIG, GOAL, GRAPH, MODEL, ROOT, START
from planning.planning_node import PlanningNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '224':
        parser.error('Run this synthetic probe with ROS_DOMAIN_ID=224')
    rclpy.init(args=['--ros-args', '-p', f'graph_path:={GRAPH}',
                     '-p', f'model_path:={MODEL}', '-p', f'config_path:={CONFIG}',
                     '-p', f'workspace_path:={ROOT / "src/planning/data/raw_workspace.npz"}'])
    node = feeder = executor = None
    try:
        node = PlanningNode()
        node.timer.cancel()
        assert node.rrt_shadow is not None
        feeder = Node('rrt_shadow_synthetic_probe', use_global_arguments=False)
        joint_pub = feeder.create_publisher(JointState, '/joint_states', 10)
        human_pub = feeder.create_publisher(HumanCapsuleArray, '/vision/human/geometry', 10)
        obstacle_pub = feeder.create_publisher(PoseArray, '/simulation/obstacle_poses', 10)
        events, commands = [], []
        subscriptions = [
            feeder.create_subscription(String, '/planning/rrt_shadow',
                                       lambda msg: events.append(json.loads(msg.data)), 10),
            feeder.create_subscription(JointTrajectory, '/motion/joint_path',
                                       lambda msg: commands.append('EXECUTE'), 10),
            feeder.create_subscription(Bool, '/motion/stop_requested',
                                       lambda msg: commands.append('STOP_OR_RELEASE'), 10),
        ]
        checker = node.planner.collision_filter
        target_world = checker.data.mocap_pos[
            checker.obstacle_geoms['target']['mocap_id']].copy()
        node.manager.update_target(np.array([.5, -.1, -.25]), target_world)
        node.manager.update_motion_status('IDLE')

        def publish():
            stamp = feeder.get_clock().now().to_msg()
            joint = JointState()
            joint.header.stamp = stamp
            joint.name = node.joint_names
            joint.position = START.tolist()
            joint_pub.publish(joint)
            human = HumanCapsuleArray()
            human.header.stamp = stamp
            human.header.frame_id = node.base_frame
            human.valid = True
            capsule = HumanCapsule()
            capsule.body_id = capsule.segment_id = 0
            capsule.start.x = capsule.start.y = capsule.start.z = 10.
            capsule.end.x = capsule.end.y = 10.
            capsule.end.z = 11.
            capsule.radius = .1
            human.capsules = [capsule]
            human_pub.publish(human)
            obstacles = PoseArray()
            obstacles.header.frame_id = 'mujoco_world'
            for name in node.obstacle_names:
                mocap = checker.obstacle_geoms[name]['mocap_id']
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    float(v) for v in checker.data.mocap_pos[mocap])
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
                    float(v) for v in checker.data.mocap_quat[mocap])
                obstacles.poses.append(pose)
            obstacle_pub.publish(obstacles)
            node.monitor_humans()

        timer = feeder.create_timer(.02, publish)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(feeder)
        until = time.monotonic() + 5.
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
            if (node.manager.feedback_ready() and node.manager.obstacle_data_ready()
                    and checker.human_data_ready()):
                break
        assert checker.human_data_ready(), 'No synthetic human received through DDS'
        assert node.manager.feedback_ready(), 'No joint feedback received through DDS'
        node.manager.last_graph_failure = dict(
            sequence=1, target_generation=node.manager.target_generation,
            start=START.copy(), goal=GOAL.copy())
        began = time.monotonic()
        feedback_stamps, human_stamps, spin_times = set(), set(), []
        while time.monotonic() - began < 8.:
            spin_began = time.perf_counter()
            executor.spin_once(timeout_sec=.005)
            spin_times.append(time.perf_counter() - spin_began)
            if node.rrt_shadow.future is not None:
                feedback_stamps.add(node.manager.feedback_time)
                human_stamps.add(node.human_monitor.stamp_ns)
            if events and events[-1].get('disposition') == 'VALID_FOR_CURRENT_SCENE_NO_DISPATCH':
                break
        assert events and events[-1].get('disposition') == (
            'VALID_FOR_CURRENT_SCENE_NO_DISPATCH'), events
        assert len(feedback_stamps) > 1 and len(human_stamps) > 1
        assert not commands, commands
        report = dict(scope=__doc__, ros_domain=224, events=events, parent_pid=os.getpid(),
                      feedback_updates_while_worker_active=len(feedback_stamps),
                      human_updates_while_worker_active=len(human_stamps),
                      max_spin_once_s=max(spin_times), elapsed_s=time.monotonic() - began,
                      motion_commands=commands, normal_astar_timer_cancelled=True)
        files = [Path(__file__), CONFIG, MODEL,
                 ROOT / 'src/planning/planning/rrt_shadow.py',
                 ROOT / 'src/planning/planning/planning_node.py',
                 ROOT / 'src/planning/planning/planning_manager.py']
        report['source_sha256'] = {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(report, indent=2))
        # Keep ROS handles alive for the duration of the experiment.
        assert subscriptions and timer
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
