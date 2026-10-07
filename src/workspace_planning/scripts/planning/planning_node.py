#!/usr/bin/env python3

import sys
from pathlib import Path

import numpy as np
import rclpy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from visualization_msgs.msg import Marker


SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent

sys.path.insert(
    0,
    str(SCRIPTS_DIR),
)

from config.robot_config import RobotConfig
from planning.a_star import AStarPlanner


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

        self.current_eof = None
        self.target_position = None

        self.last_plan_target = None

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

        self.a_star_path_publisher = (
            self.create_publisher(
                PathMessage,
                "/planning/a_star_path",
                10,
            )
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
        self.publish_gng()

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

    def target_callback(self, message):
        target = np.array(
            [
                message.x,
                message.y,
                message.z,
            ],
            dtype=np.float64,
        )

        self.target_position = target

        self.publish_target(
            self.target_position
        )

        self.get_logger().info(
            "New target: "
            f"{np.round(target, 3).tolist()}"
        )

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
        if self.last_plan_target is None:
            return True

        target_distance = np.linalg.norm(
            self.target_position
            - self.last_plan_target
        )

        return target_distance > 0.01

    def plan_path(self):
        try:
            result = self.planner.plan(
                self.current_eof,
                self.target_position,
            )

        except RuntimeError as error:
            self.get_logger().warning(
                f"A* failed: {error}"
            )

            return

        self.publish_a_star_path(
            result
        )

        self.last_plan_target = (
            self.target_position.copy()
        )

        self.get_logger().info(
            "A* planned: "
            f"{len(result['node_path'])} nodes, "
            f"length={result['path_length']:.3f} m, "
            f"start_distance="
            f"{result['start_distance']:.3f} m, "
            f"goal_distance="
            f"{result['goal_distance']:.3f} m"
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
            "gng_nodes"
        )

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

        for position in self.gng_positions:
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

        self.gng_nodes_publisher.publish(
            marker
        )

    def publish_gng_edges(self):
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
            "gng_edges"
        )

        marker.id = 0

        marker.type = Marker.LINE_LIST

        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.002

        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 0.5

        for node_a, node_b in self.gng_edges:
            node_a = int(node_a)
            node_b = int(node_b)

            if (
                node_a < 0
                or node_b < 0
                or node_a >= len(
                    self.gng_positions
                )
                or node_b >= len(
                    self.gng_positions
                )
            ):
                continue

            position_a = (
                self.gng_positions[
                    node_a
                ]
            )

            position_b = (
                self.gng_positions[
                    node_b
                ]
            )

            point_a = Point()

            point_a.x = float(
                position_a[0]
            )

            point_a.y = float(
                position_a[1]
            )

            point_a.z = float(
                position_a[2]
            )

            point_b = Point()

            point_b.x = float(
                position_b[0]
            )

            point_b.y = float(
                position_b[1]
            )

            point_b.z = float(
                position_b[2]
            )

            marker.points.append(
                point_a
            )

            marker.points.append(
                point_b
            )

        self.gng_edges_publisher.publish(
            marker
        )

    def publish_a_star_path(
        self,
        result,
    ):
        node_path = result[
            "node_path"
        ]

        graph_path = (
            self.gng_positions[
                node_path
            ]
        )

        path_positions = np.vstack(
            [
                self.current_eof,
                graph_path,
                self.target_position,
            ]
        )

        path_message = PathMessage()

        path_message.header.frame_id = (
            self.config.get_base_frame()
        )

        path_message.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for position in path_positions:
            pose = PoseStamped()

            pose.header.frame_id = (
                self.config.get_base_frame()
            )

            pose.header.stamp = (
                path_message.header.stamp
            )

            pose.pose.position.x = float(
                position[0]
            )

            pose.pose.position.y = float(
                position[1]
            )

            pose.pose.position.z = float(
                position[2]
            )

            pose.pose.orientation.w = 1.0

            path_message.poses.append(
                pose
            )

        self.a_star_path_publisher.publish(
            path_message
        )

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