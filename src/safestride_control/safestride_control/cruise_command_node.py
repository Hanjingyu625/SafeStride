"""Publish the default straight-line command used before safety limiting."""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from .safety_logic import finite_parameter


class CruiseCommandNode(Node):
    """Continuously request a bounded base speed for safety limiting."""

    def __init__(self) -> None:
        super().__init__('cruise_command')
        self.declare_parameter('speed_mps', 0.08)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        self._speed = finite_parameter(
            'speed_mps',
            self.get_parameter('speed_mps').value,
            minimum=0.0,
            maximum=1.0,
        )
        publish_rate = finite_parameter(
            'publish_rate_hz',
            self.get_parameter('publish_rate_hz').value,
            minimum=0.0,
            maximum=100.0,
            minimum_inclusive=False,
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        command_topic = str(self.get_parameter('command_topic').value)
        if not command_topic.strip() or not self._frame_id.strip():
            raise ValueError('command_topic and frame_id must not be empty')

        self._publisher = self.create_publisher(
            TwistStamped, command_topic, 10
        )
        self._timer = self.create_timer(
            1.0 / publish_rate, self._publish_command
        )
        self.get_logger().info(
            'default cruise request ready at %.3f m/s; drive enable follows '
            'live safety inputs' % self._speed
        )

    def _publish_command(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.x = self._speed
        message.twist.angular.z = 0.0
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CruiseCommandNode()
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
