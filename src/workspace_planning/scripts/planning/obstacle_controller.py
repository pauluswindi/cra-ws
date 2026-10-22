#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker


class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')

        # Konfigurasi obstacle
        defaults = {
            'obstacle_id': 1, 'obstacle_type': 'box', 'frame_id': 'base_link',
            'x': 0.3, 'y': -0.1, 'z': -0.3,
            'size_x': 0.20, 'size_y': 0.05, 'size_z': 0.09,
            'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in defaults}
        self._validate(self.values)
        self.active = True

        self.publisher = self.create_publisher(Marker, '/workspace/obstacle', 10)
        self.position_sub = self.create_subscription(Point, '~/set_position', self.position_callback, 10)
        self.remove_sub = self.create_subscription(Empty, '~/remove', self.remove_callback, 10)
        self.enable_sub = self.create_subscription(Empty, '~/enable', self.enable_callback, 10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.timer = self.create_timer(0.2, self.publish_obstacle)
        self.publish_obstacle()
        self.get_logger().info(f"Obstacle controller started: ID={self.values['obstacle_id']}")

    @staticmethod
    def _validate(values):
        if int(values['obstacle_id']) < 0:
            raise ValueError('Obstacle ID must be non-negative')
        if str(values['obstacle_type']).lower() not in ('box', 'sphere', 'cylinder'):
            raise ValueError('Unsupported obstacle type')
        numeric = ['x', 'y', 'z', 'size_x', 'size_y', 'size_z', 'qx', 'qy', 'qz', 'qw']
        if not all(math.isfinite(float(values[k])) for k in numeric):
            raise ValueError('Obstacle parameters must be finite')
        if any(float(values[k]) <= 0 for k in ('size_x', 'size_y', 'size_z')):
            raise ValueError('Obstacle size must be greater than zero')
        if sum(float(values[k]) ** 2 for k in ('qx', 'qy', 'qz', 'qw')) < 1e-12:
            raise ValueError('Invalid quaternion')

    def parameter_callback(self, parameters):
        candidate = self.values.copy()
        for parameter in parameters:
            if parameter.name in ('obstacle_id', 'frame_id'):
                return SetParametersResult(successful=False, reason='ID and frame are immutable')
            candidate[parameter.name] = parameter.value
        try:
            self._validate(candidate)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.values = candidate
        return SetParametersResult(successful=True)

    def position_callback(self, message):
        candidate = self.values.copy()
        candidate.update(x=float(message.x), y=float(message.y), z=float(message.z))
        try:
            self._validate(candidate)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.values = candidate
        self.active = True
        self.publish_obstacle()

    def remove_callback(self, _message):
        self.active = False
        marker = Marker()
        marker.header.frame_id = str(self.values['frame_id'])
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacles'
        marker.id = int(self.values['obstacle_id'])
        marker.action = Marker.DELETE
        self.publisher.publish(marker)

    def enable_callback(self, _message):
        self.active = True
        self.publish_obstacle()

    def publish_obstacle(self):
        if not self.active:
            return
        d = self.values
        marker = Marker()
        marker.header.frame_id = str(d['frame_id'])
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacles'
        marker.id = int(d['obstacle_id'])
        marker.type = {'box': Marker.CUBE, 'sphere': Marker.SPHERE,
                       'cylinder': Marker.CYLINDER}[str(d['obstacle_type']).lower()]
        marker.action = Marker.ADD
        marker.pose.position.x = float(d['x'])
        marker.pose.position.y = float(d['y'])
        marker.pose.position.z = float(d['z'])
        q = [float(d[k]) for k in ('qx', 'qy', 'qz', 'qw')]
        length = math.sqrt(sum(value ** 2 for value in q))
        marker.pose.orientation.x, marker.pose.orientation.y, \
            marker.pose.orientation.z, marker.pose.orientation.w = (v / length for v in q)
        marker.scale.x = float(d['size_x'])
        marker.scale.y = float(d['size_y'])
        marker.scale.z = float(d['size_z'])

        # Warna cyan, berbeda dari target kuning
        marker.color.r = 0.10
        marker.color.g = 0.75
        marker.color.b = 0.70
        marker.color.a = 0.85
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
