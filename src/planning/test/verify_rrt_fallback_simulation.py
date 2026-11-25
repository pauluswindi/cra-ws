"""Native Planning -> C++ Motion -> MuJoCo test on isolated domain 228.

Use a temporary disconnected two-node roadmap and real physics/controller.
The viewer is headless by default; --viewer opens the real MuJoCo window. Vision
geometry is synthetic; --changing-human changes a distant capsule every frame
to exercise revision handoff. This is not live Kinect or near-human avoidance.
"""

import argparse
from contextlib import nullcontext
import hashlib
import importlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch

import mujoco
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from msgs.msg import HumanCapsule, HumanCapsuleArray
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray
import yaml

from benchmark_rrt_connect import CONFIG, FAR, GRAPH, MODEL, ROOT, START, configure
from planning.algorithms.collision_filter import CollisionFilter
from planning.planning_node import PlanningNode
from planning.visualization.projection import project_configuration
from simulation.frame_transform import FrameTransform


GOAL = START + [.9, 0., 0., 0., 0.]


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


def simulation(viewer=False):
    from simulation.robot_simulation import RobotSimulation

    rclpy.init(args=[])
    robot = RobotSimulation()
    robot.data.qpos[robot.qpos_ids] = START
    robot.data.qvel[:] = 0.
    mujoco.mj_forward(robot.model, robot.data)
    robot.q_des = robot.hold_position_target()
    robot.data.ctrl[robot.actuator_ids] = robot.q_des
    try:
        context = (nullcontext() if viewer else
                   patch('mujoco.viewer.launch_passive', return_value=HeadlessViewer()))
        with context:
            robot.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


def fixture(directory):
    config = yaml.safe_load(CONFIG.read_text())
    assert config['robot']['mode'] == 'simulation', 'Never run this probe against a physical robot'
    checker = CollisionFilter(MODEL, config['robot']['joints'], safety_margin=.005)
    configure(checker, FAR)
    eof = [project_configuration(checker.model, checker.data, checker.qpos_addresses,
                                 checker.base_id, checker.model.body('end_effector').id, q)
           for q in (START, GOAL)]
    targets = []
    for point in eof:
        for direction in np.vstack((np.eye(3), -np.eye(3))):
            target = point + .08 * direction
            base = checker.data.xpos[checker.base_id]
            rotation = checker.data.xmat[checker.base_id].reshape(3, 3)
            world = base + rotation @ FrameTransform.robot_base_to_mujoco_base(target)
            checker.set_obstacle_pose('target', world)
            if checker.is_edge_valid(START, GOAL):
                targets.append(target)
                break
        else:
            raise AssertionError('No valid target-box placement for fixture')
    with np.load(GRAPH, allow_pickle=False) as source:
        graph = {key: source[key] for key in source.files}
    graph.update(joint_q=np.array([START, GOAL]), joint_eof=np.array(eof),
                 joint_edges=np.empty((0, 2), dtype=int), joint_costs=np.empty(0),
                 joint_node_ids=np.array([0, 1]), joint_sample_indices=np.array([0, 1]))
    path = directory / 'disconnected_graph.npz'
    np.savez(path, **graph)
    return path, targets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--viewer', action='store_true', help='Show the native MuJoCo viewer')
    parser.add_argument('--astar-mode', choices=('blocking', 'incremental'),
                        help='Override only a temporary YAML copy for this isolated probe')
    parser.add_argument('--planner-mode', choices=('gng', 'rrt', 'combined'),
                        help='Primary planner override in the temporary trial YAML')
    parser.add_argument('--connected-graph', action='store_true',
                        help='Use the same valid graph edge for a three-mode comparison')
    parser.add_argument('--rrt-visual-mode', choices=('live', 'playback'))
    parser.add_argument('--changing-human', action='store_true',
                        help='Continuously change distant synthetic geometry during RRT handoff')
    parser.add_argument('--goal-gap', action='store_true',
                        help='Remove the graph goal and require a bounded IK goal before RRT')
    parser.add_argument('--astar-max-operations', type=int,
                        help='Test-only YAML override to exercise several incremental slices')
    parser.add_argument('--primary-delay-ms', type=float, default=0.,
                        help='Test-only delay after the first real A* search (0..1500 ms)')
    args = parser.parse_args()
    if not np.isfinite(args.primary_delay_ms) or not 0 <= args.primary_delay_ms <= 1500:
        parser.error('--primary-delay-ms must be from 0 to 1500')
    selected_mode = args.astar_mode or yaml.safe_load(CONFIG.read_text()).get(
        'planning', {}).get('astar', {}).get('mode', 'blocking')
    selected_planner = args.planner_mode or yaml.safe_load(CONFIG.read_text()).get(
        'planning', {}).get('planner_mode', 'combined')
    if selected_planner == 'gng' and not args.connected_graph:
        parser.error('This native completion probe requires --connected-graph for gng mode')
    if args.goal_gap and args.connected_graph:
        parser.error('--goal-gap and --connected-graph are separate controlled fixtures')
    if args.primary_delay_ms and selected_mode != 'blocking':
        parser.error('--primary-delay-ms applies only to blocking mode')
    if args.astar_max_operations is not None and (
        selected_mode != 'incremental' or not 1 <= args.astar_max_operations <= 10000
    ):
        parser.error('--astar-max-operations requires incremental mode and 1..10000')
    if (os.environ.get('ROS_DOMAIN_ID') != '228'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        parser.error('Use isolated ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1')
    if yaml.safe_load(CONFIG.read_text())['robot']['mode'] != 'simulation':
        parser.error('This probe requires robot.mode: simulation')
    if args.simulate:
        simulation(args.viewer)
        return
    if args.output is None:
        parser.error('--output is required')
    loaded_paths = {}
    for name in ('planning_node', 'planning_manager', 'path_planner', 'goal_selection',
                 'astar_execution', 'planner_modes',
                 'search_steps', 'rrt_shadow', 'rrt_fallback', 'rrt_trace',
                 'visualization.rrt_animation', 'visualization.rrt_display', 'algorithms.a_star',
                 'algorithms.rrt_connect'):
        module = importlib.import_module('planning.' + name)
        loaded = Path(module.__file__)
        source = ROOT / 'src/planning/planning' / (name.replace('.', '/') + '.py')
        assert loaded.read_bytes() == source.read_bytes(), 'Rebuild stale package: ' + name
        loaded_paths[name] = str(loaded)
    children, logs = [], []
    node = feeder = executor = None
    with tempfile.TemporaryDirectory(prefix='cra-rrt-fallback-') as temporary:
        directory = Path(temporary)
        trial_config = CONFIG
        if (args.astar_mode or args.astar_max_operations is not None or args.goal_gap
                or args.planner_mode or args.connected_graph or args.rrt_visual_mode):
            configuration = yaml.safe_load(CONFIG.read_text())
            configuration['planning']['planner_mode'] = selected_planner
            if args.rrt_visual_mode:
                configuration['planning']['rrt'].setdefault('visualization', {})['mode'] = (
                    args.rrt_visual_mode)
            settings = configuration['planning'].setdefault('astar', {})
            settings['mode'] = selected_mode
            if args.astar_max_operations is not None:
                settings['max_operations_per_slice'] = args.astar_max_operations
            if args.goal_gap:
                configuration['planning']['joint_graph']['max_joint_delta'] = .2
                configuration['planning'].setdefault('goal_selection', {}).setdefault(
                    'ik_fallback', {})['enabled'] = True
            trial_config = directory / 'robot.yaml'
            trial_config.write_text(yaml.safe_dump(configuration))
        graph, targets = fixture(directory)
        if args.connected_graph:
            with np.load(graph, allow_pickle=False) as source:
                connected = {key: source[key] for key in source.files}
            connected['joint_edges'] = np.array([[0, 1]], dtype=int)
            connected['joint_costs'] = np.array([np.linalg.norm(GOAL - START)])
            np.savez(graph, **connected)
        if args.goal_gap:
            with np.load(graph, allow_pickle=False) as source:
                sparse = {key: source[key] for key in source.files}
            for key in ('joint_q', 'joint_eof', 'joint_node_ids', 'joint_sample_indices'):
                sparse[key] = sparse[key][:1]
            np.savez(graph, **sparse)
        try:
            simulation_command = [sys.executable, str(Path(__file__).resolve()), '--simulate']
            if args.viewer:
                simulation_command.append('--viewer')
            for command in (simulation_command,
                            ['ros2', 'launch', 'cra', 'motion.launch.py']):
                log = tempfile.TemporaryFile(mode='w+')
                logs.append(log)
                children.append(subprocess.Popen(
                    command, stdout=log, stderr=log, start_new_session=True))
            rclpy.init(args=['--ros-args', '-p', f'graph_path:={graph}',
                             '-p', f'model_path:={MODEL}',
                             '-p', f'config_path:={trial_config}',
                             '-p', 'workspace_path:='
                             + str(ROOT / 'src/planning/data/raw_workspace.npz')])
            node = PlanningNode()
            assert node.planner_mode == selected_planner
            assert node.rrt_mode == ('off' if selected_planner == 'gng' else 'fallback')
            expected_source = ('RRT_CONNECT' if selected_planner == 'rrt'
                               or not args.connected_graph else 'GNG_ASTAR')
            assert node.manager.astar_config['mode'] == selected_mode
            node.timer.cancel()
            primary_calls = 0
            if args.primary_delay_ms:
                original_plan = node.planner.plan

                def delayed_primary(*arguments):
                    nonlocal primary_calls
                    primary_calls += 1
                    planned = original_plan(*arguments)
                    if primary_calls == 1:
                        # Reproduce an occupied single-threaded callback without
                        # faking A* failure or relaxing any freshness deadline.
                        time.sleep(args.primary_delay_ms / 1000.)
                    return planned

                node.planner.plan = delayed_primary
            feeder = Node('rrt_fallback_integration_probe', use_global_arguments=False)
            human_pub = feeder.create_publisher(HumanCapsuleArray, '/vision/human/geometry', 10)
            target_pub = feeder.create_publisher(Point, '/simulation/set_target', 10)
            events, trajectories, stops, motion, statuses = [], [], [], [], []
            visual_statuses = []
            marker_points = {layer: 0 for layer in ('start_tree', 'goal_tree', 'final_path')}
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            handles = [
                feeder.create_subscription(String, '/planning/rrt/status',
                                           lambda m: visual_statuses.append(m.data), 10),
                feeder.create_subscription(String, '/planning/rrt/events',
                                           lambda m: events.append(json.loads(m.data)), 10),
                feeder.create_subscription(JointTrajectory, '/motion/joint_path',
                                           trajectories.append, 10),
                feeder.create_subscription(Bool, '/motion/stop_requested',
                                           lambda m: stops.append((time.monotonic(), m.data)), 10),
                feeder.create_subscription(String, '/motion/execution_status',
                                           lambda m: motion.append(m.data), qos),
                feeder.create_subscription(String, '/planning/status',
                                           lambda m: statuses.append(m.data), qos),
            ]

            def markers(layer, message):
                marker_points[layer] = max(marker_points[layer], sum(
                    len(m.points) for m in message.markers if m.action == m.ADD))

            handles += [feeder.create_subscription(
                MarkerArray, '/planning/rrt/' + layer,
                lambda m, name=layer: markers(name, m), qos) for layer in marker_points]
            target, valid, send_target = targets[1], True, True
            live_updates_during_search = 0
            human_frame = 0

            def publish():
                nonlocal live_updates_during_search, human_frame
                human_frame += 1
                job = node.manager.graph_search
                if job is not None and not job.done and (
                    node.planner.collision_filter.human_received_at is not None
                    and job.planner.collision_filter.human_received_at is not None
                    and node.planner.collision_filter.human_received_at
                    > job.planner.collision_filter.human_received_at
                ):
                    live_updates_during_search += 1
                message = HumanCapsuleArray()
                message.header.stamp = feeder.get_clock().now().to_msg()
                message.header.frame_id = 'base_link'
                message.valid = valid
                capsule = HumanCapsule()
                capsule.body_id = capsule.segment_id = 0
                capsule.start.x = capsule.start.y = capsule.start.z = 10.
                capsule.end.x = capsule.end.y = 10.
                capsule.end.z = 11.
                if args.changing_human:
                    # Small, bounded changes still produce a new revision;
                    # the capsule stays far from every robot link.
                    offset = .01 * np.sin(human_frame * .7)
                    capsule.start.x += offset
                    capsule.end.x += offset
                capsule.radius = .1
                message.capsules = [capsule]
                human_pub.publish(message)
                if send_target:
                    target_pub.publish(Point(
                        x=float(target[0]), y=float(target[1]), z=float(target[2])))
                node.monitor_humans()

            timer = feeder.create_timer(.05, publish)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(feeder)

            def wait_for(predicate, timeout=15.):
                until = time.monotonic() + timeout
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=.005)
                    if any(p.poll() is not None for p in children):
                        raise AssertionError('Native simulation/Motion process exited')
                    if predicate():
                        return
                raise AssertionError(dict(status=node.manager.status,
                                          reason=node.manager.rrt_last_reason,
                                          motion=motion, events=events, statuses=statuses))

            wait_for(lambda: (node.manager.feedback_ready() and node.manager.obstacle_data_ready()
                              and node.manager.rrt_human_ready()
                              and node.manager.motion_status == 'IDLE'
                              and node.manager.target_base is not None
                              and np.max(np.abs(node.manager.target_base - target)) < 1e-6))
            send_target = False
            node.timer.reset()
            began = time.monotonic()
            wait_for(lambda: ('COMPLETED' in motion and len(trajectories) >= 1
                              and node.manager.path_source == expected_source), timeout=25.)
            selected_goal = np.array(trajectories[0].points[-1].positions)
            first_selection = dict(node.planner.goal_selector.report or {})
            first_handoff = node.manager.rrt_handoff_validation
            if args.goal_gap:
                assert first_selection['source'] == 'IK', first_selection
                assert first_selection['graph_outcome'] == 'NO_GRAPH_SAMPLES', first_selection
            else:
                assert np.max(np.abs(selected_goal - GOAL)) < 1e-6
            error = float(np.max(np.abs(node.manager.current_q - selected_goal)))
            assert error <= .03, error
            if expected_source == 'RRT_CONNECT':
                assert 'RRT_PATH_READY' in statuses, statuses
                validated_events = {'FALLBACK_VALIDATED', 'CANDIDATE_REVALIDATION_REQUIRED'}
                assert any(e['event'] in validated_events for e in events), events
                if args.changing_human:
                    assert first_handoff and first_handoff['reason'] == 'NONE', first_handoff
                    assert first_handoff['from_revision'] != first_handoff['to_revision']
                trigger = 'PRIMARY_RRT' if selected_planner == 'rrt' else 'NO_GRAPH_PATH'
                assert any(e.get('trigger') == trigger for e in events), events
            else:
                assert not any(e['event'] == 'SEARCH_SUBMITTED' for e in events), events
            if selected_mode == 'incremental':
                expected_status = ('GOAL_SEARCHING' if selected_planner == 'rrt'
                                   else 'ASTAR_SEARCHING')
                assert expected_status in statuses, statuses
                assert node.manager.last_graph_search['slices'] > 0
                if args.astar_max_operations is not None and selected_planner != 'rrt':
                    assert live_updates_during_search > 0, 'No input callback between slices'
            if expected_source == 'RRT_CONNECT' and not args.changing_human:
                assert all(marker_points.values()), marker_points
            if args.primary_delay_ms > 1000:
                assert 'RRT_WAITING_FOR_FEEDBACK' in statuses, statuses
                assert primary_calls == 1, (
                    f'Primary search repeated before first RRT goal: {primary_calls}')
            first_primary_calls = primary_calls if args.primary_delay_ms else None
            first_duration = time.monotonic() - began
            if node.rrt_shadow is not None:
                wait_for(lambda: time.monotonic() - node.rrt_shadow.last_started
                         > node.rrt_shadow.settings['retry_interval'] + .1)
            first_count = len(trajectories)
            target = targets[0]
            send_target = True
            wait_for(lambda: np.max(np.abs(node.manager.target_base - target)) < 1e-6)
            send_target = False
            wait_for(lambda: len(trajectories) > first_count, timeout=12.)
            valid = False
            lost_at, count_at_loss = time.monotonic(), len(trajectories)
            wait_for(lambda: (any(t >= lost_at and stop for t, stop in stops)
                              and node.manager.motion_status == 'STOPPED'), timeout=5.)
            until = time.monotonic() + .5
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.01)
            assert len(trajectories) == count_at_loss
            assert not any(t >= lost_at and not stop for t, stop in stops)
            first_points = np.array([point.positions for point in trajectories[0].points])
            report = dict(scope=__doc__, ros_domain=228, planner_mode=selected_planner,
                          roadmap_nodes=1 if args.goal_gap else 2,
                          roadmap_edges=1 if args.connected_graph else 0,
                          connected_graph_fixture=args.connected_graph,
                          test_goal_gap=args.goal_gap, first_goal_selection=first_selection,
                          changing_human_geometry=args.changing_human,
                          visual_mode=node.rrt_display.settings['mode'],
                          visual_statuses=visual_statuses,
                          live_progress_packets=(node.rrt_shadow.live_packets_received
                                                 if node.rrt_shadow is not None else 0),
                          first_rrt_handoff_validation=first_handoff,
                          astar_mode=selected_mode, astar_search=node.manager.last_graph_search,
                          astar_configuration=node.manager.astar_config,
                          live_updates_during_astar=live_updates_during_search,
                          fixture_start=START.tolist(), fixture_goal=selected_goal.tolist(),
                          path_source=expected_source, first_goal_max_error_rad=error,
                          first_path_points=len(first_points),
                          first_path_length_rad=float(np.linalg.norm(
                              np.diff(first_points, axis=0), axis=1).sum()),
                          first_goal_duration_s=first_duration,
                          test_primary_delay_ms=args.primary_delay_ms,
                          first_goal_primary_search_calls=first_primary_calls,
                          input_refresh_wait_observed='RRT_WAITING_FOR_FEEDBACK' in statuses,
                          trajectory_messages=len(trajectories), rrt_events=events,
                          rrt_marker_points=marker_points,
                          planning_statuses=statuses, motion_statuses=motion,
                          camera_loss_caused_stop=True,
                          no_execute_or_release_after_camera_loss=True,
                          measured_q_after_stop=node.manager.current_q.tolist(),
                          native_mujoco_physics=True, native_cpp_motion=True,
                          interactive_viewer=args.viewer, synthetic_human_input=True)
            report['loaded_package_paths'] = loaded_paths
            files = [Path(__file__), CONFIG, MODEL,
                     ROOT / 'src/planning/planning/planning_manager.py',
                     ROOT / 'src/planning/planning/planning_node.py',
                     ROOT / 'src/planning/planning/astar_execution.py',
                     ROOT / 'src/planning/planning/goal_selection.py',
                     ROOT / 'src/planning/planning/planner_modes.py',
                     ROOT / 'src/planning/planning/search_steps.py',
                     ROOT / 'src/planning/planning/algorithms/a_star.py',
                     ROOT / 'src/planning/planning/algorithms/rrt_connect.py',
                     ROOT / 'src/planning/planning/algorithms/collision_filter.py',
                     ROOT / 'src/planning/planning/path_planner.py',
                     ROOT / 'src/planning/planning/rrt_fallback.py',
                     ROOT / 'src/planning/planning/rrt_shadow.py',
                     ROOT / 'src/planning/planning/rrt_trace.py',
                     ROOT / 'src/planning/planning/visualization/rrt_animation.py',
                     ROOT / 'src/planning/planning/visualization/rrt_display.py']
            report['source_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(
                p.read_bytes()).hexdigest() for p in files}
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
            assert handles and timer
        except Exception:
            for log in logs:
                log.seek(0)
                print(log.read()[-14000:])
            raise
        finally:
            if executor is not None:
                executor.shutdown()
            if feeder is not None:
                feeder.destroy_node()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for child in children:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=6.)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
            for log in logs:
                log.close()


if __name__ == '__main__':
    main()
