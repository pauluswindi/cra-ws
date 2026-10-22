#!/usr/bin/env python3

from pathlib import Path

import mujoco
import numpy as np

from geometry_msgs.msg import Point
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from simulation.frame_transform import FrameTransform


class PlanningVisualizer:
    def __init__(
        self,
        node,
        workspace_path,
        graph_path,
        model_path,
        joint_names,
        base_body,
        end_effector_body,
        frame_id="base_link",
    ):
        self.node = node
        self.frame_id = frame_id
        self.joint_names = list(joint_names)

        if len(self.joint_names) != 5:
            raise ValueError("Expected five robot joints")

        self.workspace = self.load_workspace(workspace_path)
        self.gng_nodes, self.gng_edges = self.load_gng(graph_path)

        self.model = mujoco.MjModel.from_xml_path(
            str(model_path)
        )
        self.data = mujoco.MjData(self.model)

        self.base_id = self.model.body(base_body).id
        self.eof_id = self.model.body(end_effector_body).id

        self.qpos_ids = [
            int(
                self.model.jnt_qposadr[
                    self.model.joint(name).id
                ]
            )
            for name in self.joint_names
        ]

        # This limit affects RViz only, not A* planning.
        self.max_visible_joint_edges = 2000

        self.joint_graph_positions = None
        self.joint_graph_revision = None

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.workspace_pub = node.create_publisher(
            Marker,
            "/planning/workspace",
            marker_qos,
        )

        self.gng_nodes_pub = node.create_publisher(
            Marker,
            "/planning/gng_nodes",
            marker_qos,
        )

        self.gng_edges_pub = node.create_publisher(
            Marker,
            "/planning/gng_edges",
            marker_qos,
        )

        self.joint_nodes_pub = node.create_publisher(
            Marker,
            "/planning/joint_graph_nodes",
            marker_qos,
        )

        self.joint_edges_pub = node.create_publisher(
            Marker,
            "/planning/joint_graph_edges",
            marker_qos,
        )

        self.eof_pub = node.create_publisher(
            Marker,
            "/planning/eof_marker",
            marker_qos,
        )

        self.path_publishers = {
            kind: node.create_publisher(
                Marker,
                f"/planning/{kind}_path",
                marker_qos,
            )
            for kind in (
                "initial",
                "invalid",
                "replanned",
            )
        }

        self.path_colors = {
            "initial": (0.0, 0.45, 1.0),
            "invalid": (1.0, 0.10, 0.10),
            "replanned": (0.0, 0.85, 1.0),
        }

        self.path_widths = {
            "initial": 0.008,
            "invalid": 0.006,
            "replanned": 0.009,
        }

        self.joint_state_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.publish_static()

        self.node.get_logger().info(
            "PlanningVisualizer initialized"
        )

    @staticmethod
    def load_workspace(path):
        with np.load(Path(path), allow_pickle=False) as data:
            for key in (
                "eof",
                "positions",
                "eof_positions",
                "workspace",
            ):
                if key in data:
                    positions = np.asarray(
                        data[key],
                        dtype=np.float32,
                    ).copy()
                    break
            else:
                raise KeyError(
                    f"Workspace positions not found: {data.files}"
                )

        if (
            positions.ndim != 2
            or positions.shape[1] != 3
            or not np.isfinite(positions).all()
        ):
            raise ValueError("Invalid workspace positions")

        return positions

    @staticmethod
    def load_gng(path):
        with np.load(Path(path), allow_pickle=False) as data:
            nodes = np.asarray(
                data["positions"],
                dtype=float,
            ).copy()

            edges = np.asarray(
                data["edges"],
                dtype=np.int64,
            ).reshape(-1, 2).copy()

        if (
            nodes.ndim != 2
            or nodes.shape[1] != 3
            or not np.isfinite(nodes).all()
        ):
            raise ValueError("Invalid GNG positions")

        if len(edges) and (
            edges.min() < 0
            or edges.max() >= len(nodes)
        ):
            raise ValueError("Invalid GNG edges")

        return nodes, edges

    @staticmethod
    def make_point(position):
        return Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        )

    def make_marker(
        self,
        namespace,
        marker_type,
        size,
        color,
        alpha=1.0,
    ):
        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = (
            self.node.get_clock().now().to_msg()
        )

        marker.ns = namespace
        marker.id = 0
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = float(size)

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(alpha)

        return marker

    def publish_workspace(self):
        marker = self.make_marker(
            namespace="raw_workspace",
            marker_type=Marker.POINTS,
            size=0.004,
            color=(0.5, 0.5, 0.5),
            alpha=0.25,
        )

        marker.scale.y = 0.004

        marker.points = [
            self.make_point(position)
            for position in self.workspace
        ]

        self.workspace_pub.publish(marker)

    def publish_gng_nodes(self):
        marker = self.make_marker(
            namespace="gng_nodes",
            marker_type=Marker.POINTS,
            size=0.008,
            color=(0.0, 1.0, 0.0),
            alpha=0.85,
        )

        marker.scale.y = 0.008

        marker.points = [
            self.make_point(position)
            for position in self.gng_nodes
        ]

        self.gng_nodes_pub.publish(marker)

    def publish_gng_edges(self):
        marker = self.make_marker(
            namespace="gng_edges",
            marker_type=Marker.LINE_LIST,
            size=0.0015,
            color=(1.0, 0.45, 0.0),
            alpha=0.35,
        )

        for a, b in self.gng_edges:
            marker.points.append(
                self.make_point(self.gng_nodes[a])
            )
            marker.points.append(
                self.make_point(self.gng_nodes[b])
            )

        self.gng_edges_pub.publish(marker)

    def publish_static(self):
        self.publish_workspace()
        self.publish_gng_nodes()
        self.publish_gng_edges()

        self.node.get_logger().info(
            "Workspace GNG published: "
            f"{len(self.gng_nodes)} nodes, "
            f"{len(self.gng_edges)} edges"
        )

    def forward_kinematics(self, joints):
        joints = np.asarray(joints, dtype=float)

        if (
            joints.shape != (5,)
            or not np.isfinite(joints).all()
        ):
            raise ValueError("Invalid joint configuration")

        self.data.qpos[self.qpos_ids] = joints

        mujoco.mj_forward(self.model, self.data)

        base_position = self.data.xpos[self.base_id]

        base_rotation = self.data.xmat[
            self.base_id
        ].reshape(3, 3)

        eof_position = self.data.xpos[self.eof_id]

        position_mujoco_base = (
            base_rotation.T
            @ (eof_position - base_position)
        )

        return np.asarray(
            FrameTransform.mujoco_base_to_robot_base(
                position_mujoco_base
            ),
            dtype=float,
        )

    def joint_state_callback(self, message):
        if len(message.name) != len(message.position):
            return

        if len(set(message.name)) != len(message.name):
            return

        positions = dict(
            zip(message.name, message.position)
        )

        if any(
            name not in positions
            for name in self.joint_names
        ):
            return

        joints = np.asarray(
            [
                positions[name]
                for name in self.joint_names
            ],
            dtype=float,
        )

        if not np.isfinite(joints).all():
            return

        eof_position = self.forward_kinematics(joints)

        self.publish_eof(eof_position)

    def publish_eof(self, position):
        marker = self.make_marker(
            namespace="current_eof",
            marker_type=Marker.SPHERE,
            size=0.030,
            color=(1.0, 0.0, 0.0),
            alpha=1.0,
        )

        marker.scale.y = 0.030
        marker.scale.z = 0.030

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])

        self.eof_pub.publish(marker)

    def publish_filtered_graph(self, filtered_graph):
        joint_q = np.asarray(
            filtered_graph["joint_q"],
            dtype=float,
        )

        joint_edges = np.asarray(
            filtered_graph["joint_edges"],
            dtype=np.int64,
        ).reshape(-1, 2)

        valid_nodes = np.asarray(
            filtered_graph["valid_node_mask"],
            dtype=bool,
        )

        valid_edges = np.asarray(
            filtered_graph["valid_edge_mask"],
            dtype=bool,
        )

        revision = filtered_graph["environment_revision"]

        if (
            joint_q.ndim != 2
            or joint_q.shape[1] != 5
            or not np.isfinite(joint_q).all()
        ):
            raise ValueError("Invalid joint graph")

        if (
            valid_nodes.shape != (len(joint_q),)
            or valid_edges.shape != (len(joint_edges),)
        ):
            raise ValueError("Invalid graph validity masks")

        if len(joint_edges) and (
            joint_edges.min() < 0
            or joint_edges.max() >= len(joint_q)
        ):
            raise ValueError("Invalid joint edge indices")

        if len(joint_edges):
            a = joint_edges[:, 0]
            b = joint_edges[:, 1]

            valid_edges = (
                valid_edges
                & valid_nodes[a]
                & valid_nodes[b]
            )

        if (
            self.joint_graph_positions is None
            or len(self.joint_graph_positions) != len(joint_q)
        ):
            self.joint_graph_positions = np.asarray(
                [
                    self.forward_kinematics(joints)
                    for joints in joint_q
                ],
                dtype=float,
            )

        self.publish_joint_graph_nodes(
            self.joint_graph_positions,
            valid_nodes,
        )

        self.publish_joint_graph_edges(
            self.joint_graph_positions,
            joint_edges,
            valid_edges,
        )

        self.joint_graph_revision = revision

        self.node.get_logger().info(
            "Filtered joint graph updated: "
            f"{np.count_nonzero(valid_nodes)} valid nodes, "
            f"{np.count_nonzero(valid_edges)} valid edges"
        )

    def publish_joint_graph_nodes(
        self,
        positions,
        valid_nodes,
    ):
        marker = self.make_marker(
            namespace="joint_graph_nodes",
            marker_type=Marker.POINTS,
            size=0.005,
            color=(0.0, 1.0, 0.0),
            alpha=0.65,
        )

        marker.scale.y = 0.005

        marker.points = [
            self.make_point(position)
            for position in positions[valid_nodes]
        ]

        self.joint_nodes_pub.publish(marker)

    def publish_joint_graph_edges(
        self,
        positions,
        edges,
        valid_edges,
    ):
        marker = self.make_marker(
            namespace="joint_graph_edges",
            marker_type=Marker.LINE_LIST,
            size=0.001,
            color=(1.0, 0.45, 0.0),
            alpha=0.30,
        )

        visible_edges = edges[valid_edges]

        if (
            len(visible_edges) >
            self.max_visible_joint_edges
        ):
            indices = np.linspace(
                0,
                len(visible_edges) - 1,
                self.max_visible_joint_edges,
                dtype=int,
            )

            visible_edges = visible_edges[indices]

        for a, b in visible_edges:
            marker.points.append(
                self.make_point(positions[a])
            )
            marker.points.append(
                self.make_point(positions[b])
            )

        self.joint_edges_pub.publish(marker)

        self.node.get_logger().info(
            f"Joint edges in RViz: {len(visible_edges)} "
            f"of {np.count_nonzero(valid_edges)} valid"
        )

    def joint_path_to_xyz(
        self,
        path,
        max_step=0.04,
    ):
        path = np.asarray(path, dtype=float)

        if (
            path.ndim != 2
            or path.shape[1] != 5
            or not np.isfinite(path).all()
        ):
            raise ValueError("Invalid joint path")

        if len(path) == 0:
            return []

        positions = []

        for start, goal in zip(
            path[:-1],
            path[1:],
        ):
            distance = np.max(
                np.abs(goal - start)
            )

            steps = max(
                1,
                int(np.ceil(distance / max_step)),
            )

            for index in range(steps):
                ratio = index / steps

                joints = (
                    start
                    + ratio * (goal - start)
                )

                positions.append(
                    self.forward_kinematics(joints)
                )

        positions.append(
            self.forward_kinematics(path[-1])
        )

        return positions

    def publish_path(self, path, kind):
        if kind not in self.path_publishers:
            raise ValueError(
                f"Unknown path kind: {kind}"
            )

        marker = self.make_marker(
            namespace=f"{kind}_path",
            marker_type=Marker.LINE_STRIP,
            size=self.path_widths[kind],
            color=self.path_colors[kind],
            alpha=0.55 if kind == "invalid" else 1.0,
        )

        marker.points = [
            self.make_point(position)
            for position in self.joint_path_to_xyz(path)
        ]

        self.path_publishers[kind].publish(marker)

    def clear_path(self, kind):
        if kind not in self.path_publishers:
            raise ValueError(
                f"Unknown path kind: {kind}"
            )

        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = (
            self.node.get_clock().now().to_msg()
        )

        marker.ns = f"{kind}_path"
        marker.id = 0
        marker.action = Marker.DELETE

        self.path_publishers[kind].publish(marker)