#!/usr/bin/env python3

import sys

from pathlib import Path

import numpy as np
import yaml

import rclpy

from geometry_msgs.msg import Point, PoseStamped

from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import Empty

from rclpy.node import Node

from visualization_msgs.msg import Marker

SCRIPT_DIR = Path(__file__).resolve().parent

SCRIPTS_DIR = SCRIPT_DIR.parent
TRAJECTORY_DIR = SCRIPTS_DIR / "trajectory"

sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(TRAJECTORY_DIR))

from config.robot_config import RobotConfig

from planning.a_star import AStarPlanner
from planning.workspace_filter import WorkspaceFilter, load_target_geometry
from trajectory_generator import TrajectoryGenerator

class PlanningNode(Node):

    def __init__(self):

        super().__init__(

            "planning_node"

        )

        package_dir = (

            SCRIPT_DIR.parent.parent

        )

        config_path = (

            package_dir

            / "config"

            / "robot.yaml"

        )

        data_dir = (

            package_dir

            / "data"

        )

        graph_path = (

            data_dir

            / "workspace_graph.npz"

        )

        raw_workspace_path = (

            data_dir

            / "raw_workspace.npz"

        )

        self.config = RobotConfig(

            config_path

        )
        with config_path.open("r", encoding="utf-8") as config_file:
            config_data = yaml.safe_load(config_file) or {}

        model_relative_path = (
            config_data.get("robot", {})
            .get("model", {})
            .get("path", "robot_model/cognitive_robotics_arm.xml")
        )
        model_path = (package_dir / model_relative_path).resolve()
        self.target_geometry = load_target_geometry(
            model_path,
            target_body_name="target",
            base_body_name=self.config.get_base_frame(),
        )

        trajectory_config = config_data.get("trajectory", {})
        self.trajectory_enabled = trajectory_config.get("enabled", True)
        self.trajectory_generator = TrajectoryGenerator(
            speed=trajectory_config.get("speed", 0.1),
            distance_step=trajectory_config.get("distance_step", 0.01),
            curve_type=trajectory_config.get("curve", "smooth_step"),
        )

        self.graph_data = np.load(

            graph_path,

            allow_pickle=True,

        )

        self.raw_workspace_data = np.load(

            raw_workspace_path,

            allow_pickle=True,

        )

        self.gng_positions = np.asarray(

            self.graph_data["positions"],

            dtype=np.float64,

        )

        self.gng_edges = np.asarray(

            self.graph_data["edges"],

            dtype=np.int64,

        )

        self.raw_workspace = np.asarray(

            self.raw_workspace_data["eof"],

            dtype=np.float64,

        )

        self.planner = AStarPlanner(
            positions=self.gng_positions,
            edges=self.gng_edges,
            q=self.graph_data["q"],
        )

        self.workspace_filter = WorkspaceFilter(
            positions=self.gng_positions,
            edges=self.gng_edges,
            safety_margin=0.01,
        )
        self.target_position = None
        self.target_filter_center = (
            self.target_geometry.initial_position
            + self.target_geometry.center_offset
        )
        self.workspace_filter.set_target_box(
            self.target_filter_center,
            self.target_geometry.size,
        )
        self.valid_node_mask, self.valid_edge_mask = self.workspace_filter.update()
        self.planner.set_graph_validity(
            self.valid_node_mask,
            self.valid_edge_mask,
        )

        self.current_eof = None
        self.last_plan_target = None
        self.graph_filter_dirty = True

        self.target_subscriber = (

            self.create_subscription(

                Point,

                "/planning/target_position",

                self.target_callback,

                10,

            )

        )

        self.eof_subscriber = (

            self.create_subscription(

                Point,

                "/planning/current_eof",

                self.eof_callback,

                10,

            )

        )

        self.obstacle_subscriber = self.create_subscription(
            Marker,
            "/workspace/obstacle",
            self.obstacle_callback,
            10,
        )

        self.skip_validation_subscriber = self.create_subscription(
            PathMessage, "/planning/validate_skip", self.validate_skip_callback, 10
        )
        self.skip_validation_publisher = self.create_publisher(
            PathMessage, "/planning/skip_validation_result", 10
        )
        self.replan_subscriber = self.create_subscription(
            Empty, "/planning/request_replan", self.request_replan_callback, 10
        )

        self.a_star_path_publisher = (

            self.create_publisher(

                PathMessage,

                "/planning/a_star_path",

                10,

            )

        )
        self.trajectory_publisher = self.create_publisher(
            PathMessage,
            "/planning/trajectory",
            10,
        )

        self.workspace_publisher = (

            self.create_publisher(

                Marker,

                "/planning/raw_workspace",

                10,

            )

        )

        self.gng_nodes_publisher = (

            self.create_publisher(

                Marker,

                "/planning/gng_nodes",

                10,

            )

        )

        self.gng_edges_publisher = (

            self.create_publisher(

                Marker,

                "/planning/gng_edges",

                10,

            )

        )

        self.blocked_nodes_publisher = self.create_publisher(
            Marker,
            "/planning/blocked_gng_nodes",
            10,
        )
        self.target_bounds_publisher = self.create_publisher(
            Marker,
            "/planning/target_bounds",
            10,
        )

        self.start_publisher = (

            self.create_publisher(

                Marker,

                "/planning/planning_start",

                10,

            )

        )

        self.target_publisher = (

            self.create_publisher(

                Marker,

                "/planning/planning_target",

                10,

            )

        )

        self.timer = self.create_timer(

            0.1,

            self.planning_callback,

        )

        self.publish_workspace()
        self._refresh_workspace_filter(publish_markers=False)
        self.publish_gng()
        self.publish_target_bounds()

        self.get_logger().info(

            "Planning node started."

        )

        self.get_logger().info(

            f"Raw workspace: "

            f"{len(self.raw_workspace)} points"

        )

        self.get_logger().info(

            f"GNG graph: "

            f"{len(self.gng_positions)} nodes, "

            f"{len(self.gng_edges)} edges"

        )
        self.get_logger().info(
            "Trajectory settings: "
            f"enabled={self.trajectory_enabled}, "
            f"speed={self.trajectory_generator.speed:.3f} m/s, "
            f"distance_step={self.trajectory_generator.distance_step:.3f} m, "
            f"curve={self.trajectory_generator.curve.curve_type}"
        )

    def target_callback(self, message):

        target = np.array(

            [

                message.x,

                message.y,

                message.z,

            ],

            dtype=np.float64,

        )

        target_changed = (
            self.target_position is None
            or not np.allclose(target, self.target_position, atol=1e-6)
        )
        self.target_position = target
        if target_changed:
            self.target_filter_center = (
                self.target_position + self.target_geometry.center_offset
            )
            bounds_changed = self.workspace_filter.set_target_box(
                self.target_filter_center,
                self.target_geometry.size,
            )
            if bounds_changed:
                self._refresh_workspace_filter()

        self.publish_target(

            self.target_position

        )

        self.get_logger().info(

            "New target: "

            f"{np.round(target, 3).tolist()}"

        )

    def obstacle_callback(self, message):
        base_frame = self.config.get_base_frame()
        frame_id = message.header.frame_id or base_frame

        if message.action in (Marker.DELETE, Marker.DELETEALL):
            if self.workspace_filter.clear_obstacle():
                self.get_logger().info("Dummy obstacle removed.")
                self._refresh_workspace_filter()
            return

        if message.action != Marker.ADD:
            return

        if frame_id != base_frame:
            self.get_logger().warning(
                f"Obstacle frame '{frame_id}' does not match '{base_frame}'. "
                "Obstacle update ignored."
            )
            return

        shape_types = {
            Marker.CUBE: "box",
            Marker.SPHERE: "sphere",
            Marker.CYLINDER: "cylinder",
        }
        shape_type = shape_types.get(message.type)
        if shape_type is None:
            self.get_logger().warning(
                "Obstacle type is unsupported. Use CUBE, SPHERE, or CYLINDER."
            )
            return

        try:
            obstacle_changed = self.workspace_filter.set_obstacle(
                shape_type=shape_type,
                center=[
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ],
                size=[message.scale.x, message.scale.y, message.scale.z],
                quaternion=[
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ],
            )
        except ValueError as error:
            self.get_logger().warning(f"Obstacle update ignored: {error}")
            return

        if not obstacle_changed:
            return

        self.get_logger().info(
            f"Obstacle updated: type={shape_type}, "
            f"position={[round(message.pose.position.x, 3), round(message.pose.position.y, 3), round(message.pose.position.z, 3)]}, "
            f"size={[round(message.scale.x, 3), round(message.scale.y, 3), round(message.scale.z, 3)]}"
        )
        self._refresh_workspace_filter()

    def _refresh_workspace_filter(self, publish_markers=True):
        self.valid_node_mask, self.valid_edge_mask = self.workspace_filter.update()
        self.planner.set_graph_validity(
            self.valid_node_mask,
            self.valid_edge_mask,
        )
        self.graph_filter_dirty = True

        valid_nodes = int(np.count_nonzero(self.valid_node_mask))
        blocked_nodes = len(self.valid_node_mask) - valid_nodes
        valid_edges = int(np.count_nonzero(self.valid_edge_mask))
        blocked_edges = len(self.valid_edge_mask) - valid_edges
        self.get_logger().info(
            f"Workspace filter: {valid_nodes} valid nodes, {blocked_nodes} blocked nodes, "
            f"{valid_edges} valid edges, {blocked_edges} blocked edges."
        )

        if publish_markers:
            self.publish_gng()
            self.publish_target_bounds()

    def eof_callback(self, message):

        self.current_eof = np.array(

            [

                message.x,

                message.y,

                message.z,

            ],

            dtype=np.float64,

        )

        self.publish_start(

            self.current_eof

        )

    def validate_skip_callback(self, request):
        response = PathMessage()
        response.header.frame_id = self.config.get_base_frame()
        if (request.header.frame_id == self.config.get_base_frame()
                and len(request.poses) == 2):
            positions = np.array([
                [p.pose.position.x, p.pose.position.y, p.pose.position.z]
                for p in request.poses
            ], dtype=np.float64)
            if (np.all(np.isfinite(positions)) and
                    not self.workspace_filter.is_segment_blocked(
                        positions[0], positions[1],
                        include_target=True, include_obstacle=True)):
                response.poses = request.poses
                self.get_logger().info("Skip segment validated as unblocked")
            else:
                self.get_logger().warning("Skip segment rejected: obstruction or invalid data")
        self.skip_validation_publisher.publish(response)

    def request_replan_callback(self, _message):
        self.get_logger().warning("Executor requested replanning from actual EoF")
        self.graph_filter_dirty = True
        if self.current_eof is not None and self.target_position is not None:
            self.plan_path()

    def planning_callback(self):

        if (

            self.current_eof is None

            or self.target_position is None

        ):

            return

        if not self._should_replan():

            return

        self.plan_path()

    def _should_replan(self):

        if self.graph_filter_dirty:
            return True

        if self.last_plan_target is None:

            return True

        target_distance = np.linalg.norm(

            self.target_position

            - self.last_plan_target

        )

        return target_distance > 0.01

    def plan_path(self):
        self.graph_filter_dirty = False
        self.last_plan_target = self.target_position.copy()

        start_connector_valid = np.array(
            [
                not self.workspace_filter.is_segment_blocked(
                    self.current_eof,
                    position,
                )
                for position in self.gng_positions
            ],
            dtype=bool,
        )
        goal_connector_valid = np.array(
            [
                not self.workspace_filter.is_segment_blocked(
                    position,
                    self.target_position,
                    include_target=False,
                    include_obstacle=True,
                )
                for position in self.gng_positions
            ],
            dtype=bool,
        )

        try:
            result = self.planner.plan(
                self.current_eof,
                self.target_position,
                start_connector_valid=start_connector_valid,
                goal_connector_valid=goal_connector_valid,
            )

            start_node_position = self.gng_positions[result["start_node"]]
            goal_node_position = self.gng_positions[result["goal_node"]]

            if self.workspace_filter.is_segment_blocked(
                self.current_eof,
                start_node_position,
            ):
                raise RuntimeError(
                    "The direct connection from the current EoF to the graph is blocked."
                )

            # The final connection may enter the target object intentionally, but it
            # must not pass through the separately published dummy obstacle.
            if self.workspace_filter.is_segment_blocked(
                goal_node_position,
                self.target_position,
                include_target=False,
                include_obstacle=True,
            ):
                raise RuntimeError(
                    "The final connection to the target intersects the dummy obstacle."
                )

        except RuntimeError as error:
            self.get_logger().warning(f"A* failed: {error}")
            return

        self.publish_a_star_path(result)

        self.get_logger().info(
            "A* planned: "
            f"{len(result['node_path'])} nodes, "
            f"length={result['path_length']:.3f} m, "
            f"start_distance={result['start_distance']:.3f} m, "
            f"goal_distance={result['goal_distance']:.3f} m"
        )

    def publish_workspace(self):

        marker = Marker()

        marker.header.frame_id = (

            self.config.get_base_frame()

        )

        marker.header.stamp = (

            self.get_clock()

            .now()

            .to_msg()

        )

        marker.ns = (

            "raw_workspace"

        )

        marker.id = 0

        marker.type = Marker.POINTS

        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.004

        marker.scale.y = 0.004

        marker.color.r = 0.5

        marker.color.g = 0.5

        marker.color.b = 0.5

        marker.color.a = 0.25

        for position in self.raw_workspace:

            point = Point()

            point.x = float(

                position[0]

            )

            point.y = float(

                position[1]

            )

            point.z = float(

                position[2]

            )

            marker.points.append(

                point

            )

        self.workspace_publisher.publish(

            marker

        )

    def publish_gng(self):

        self.publish_gng_nodes()

        self.publish_gng_edges()

    def publish_gng_nodes(self):
        stamp = self.get_clock().now().to_msg()
        frame_id = self.config.get_base_frame()

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "gng_nodes"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.012
        marker.scale.y = 0.012
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        blocked_marker = Marker()
        blocked_marker.header.frame_id = frame_id
        blocked_marker.header.stamp = stamp
        blocked_marker.ns = "blocked_gng_nodes"
        blocked_marker.id = 0
        blocked_marker.type = Marker.POINTS
        blocked_marker.action = Marker.ADD
        blocked_marker.pose.orientation.w = 1.0
        blocked_marker.scale.x = 0.016
        blocked_marker.scale.y = 0.016
        blocked_marker.color.r = 1.0
        blocked_marker.color.g = 0.0
        blocked_marker.color.b = 0.0
        blocked_marker.color.a = 1.0

        for index, position in enumerate(self.gng_positions):
            point = Point()
            point.x = float(position[0])
            point.y = float(position[1])
            point.z = float(position[2])

            if self.valid_node_mask[index]:
                marker.points.append(point)
            else:
                blocked_marker.points.append(point)

        self.gng_nodes_publisher.publish(marker)
        self.blocked_nodes_publisher.publish(blocked_marker)

    def publish_gng_edges(self):
        marker = Marker()
        marker.header.frame_id = self.config.get_base_frame()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gng_edges"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.002
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 0.5

        for edge_index, (node_a, node_b) in enumerate(self.gng_edges):
            if not self.valid_edge_mask[edge_index]:
                continue

            node_a = int(node_a)
            node_b = int(node_b)
            if (
                node_a < 0 or node_b < 0
                or node_a >= len(self.gng_positions)
                or node_b >= len(self.gng_positions)
            ):
                continue

            for position in (self.gng_positions[node_a], self.gng_positions[node_b]):
                point = Point()
                point.x = float(position[0])
                point.y = float(position[1])
                point.z = float(position[2])
                marker.points.append(point)

        self.gng_edges_publisher.publish(marker)

    def publish_target_bounds(self):
        marker = Marker()
        marker.header.frame_id = self.config.get_base_frame()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_bounds"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.workspace_filter.target_box_center[0])
        marker.pose.position.y = float(self.workspace_filter.target_box_center[1])
        marker.pose.position.z = float(self.workspace_filter.target_box_center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.workspace_filter.target_box_size[0])
        marker.scale.y = float(self.workspace_filter.target_box_size[1])
        marker.scale.z = float(self.workspace_filter.target_box_size[2])
        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.18
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.target_bounds_publisher.publish(marker)

    def publish_a_star_path(self, result):
        node_path = np.asarray(result["node_path"], dtype=np.int64)
        graph_path = self.gng_positions[node_path]

        path_positions = np.vstack(
            [
                self.current_eof,
                graph_path,
                self.target_position,
            ]
        )
        path_positions = self._remove_duplicate_positions(path_positions)

        stamp = self.get_clock().now().to_msg()
        path_message = self._create_path_message(
            path_positions,
            stamp,
        )
        self.a_star_path_publisher.publish(path_message)

        if self.trajectory_enabled:
            try:
                trajectory = self.trajectory_generator.generate(path_positions)
            except (ValueError, RuntimeError) as error:
                self.get_logger().warning(
                    f"Trajectory generation failed: {error}"
                )
                return

            trajectory_message = self._create_path_message(
                trajectory["positions"],
                stamp,
            )
            self.trajectory_publisher.publish(trajectory_message)

            self.get_logger().info(
                "Trajectory generated: "
                f"{len(trajectory['positions'])} points, "
                f"length={trajectory['total_distance']:.3f} m, "
                f"duration={trajectory['total_time']:.3f} s, "
                f"curve={self.trajectory_generator.curve.curve_type}"
            )

    def _create_path_message(self, positions, stamp):
        path_message = PathMessage()
        path_message.header.frame_id = self.config.get_base_frame()
        path_message.header.stamp = stamp

        for position in positions:
            pose = PoseStamped()
            pose.header.frame_id = self.config.get_base_frame()
            pose.header.stamp = stamp
            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2])
            pose.pose.orientation.w = 1.0
            path_message.poses.append(pose)

        return path_message

    @staticmethod
    def _remove_duplicate_positions(positions, tolerance=1e-9):
        positions = np.asarray(positions, dtype=np.float64)
        if len(positions) < 2:
            return positions

        result = [positions[0]]
        for position in positions[1:]:
            if np.linalg.norm(position - result[-1]) > tolerance:
                result.append(position)

        return np.asarray(result, dtype=np.float64)

    def publish_start(

        self,

        position,

    ):

        marker = self.create_sphere_marker(

            position,

            marker_id=0,

            scale=0.035,

        )

        marker.ns = (

            "planning_start"

        )

        marker.color.r = 0.0

        marker.color.g = 0.5

        marker.color.b = 1.0

        marker.color.a = 1.0

        self.start_publisher.publish(

            marker

        )

    def publish_target(

        self,

        position,

    ):

        marker = self.create_sphere_marker(

            position,

            marker_id=0,

            scale=0.045,

        )

        marker.ns = (

            "planning_target"

        )

        marker.color.r = 1.0

        marker.color.g = 0.0

        marker.color.b = 0.0

        marker.color.a = 1.0

        self.target_publisher.publish(

            marker

        )

    def create_sphere_marker(

        self,

        position,

        marker_id,

        scale,

    ):

        marker = Marker()

        marker.header.frame_id = (

            self.config.get_base_frame()

        )

        marker.header.stamp = (

            self.get_clock()

            .now()

            .to_msg()

        )

        marker.id = marker_id

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        marker.pose.position.x = float(

            position[0]

        )

        marker.pose.position.y = float(

            position[1]

        )

        marker.pose.position.z = float(

            position[2]

        )

        marker.pose.orientation.w = 1.0

        marker.scale.x = scale

        marker.scale.y = scale

        marker.scale.z = scale

        return marker

def main(args=None):

    rclpy.init(

        args=args

    )

    node = PlanningNode()

    try:

        rclpy.spin(

            node

        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()

if __name__ == "__main__":

    main()