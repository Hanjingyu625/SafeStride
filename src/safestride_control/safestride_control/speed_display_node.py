"""Publish km/h companions for standard ROS speed and velocity topics."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class SpeedDisplay(Node):
    """Convert SI linear speeds without changing control messages."""

    def __init__(self):
        super().__init__('speed_display')
        self._subscriptions = []
        for topic, msg_type, extract in (
            ('/gps/speed', Float32, lambda m: m.data),
            ('/gps/speed_raw', Float32, lambda m: m.data),
            ('/odom', Odometry, lambda m: m.twist.twist.linear.x),
            ('/cmd_vel', TwistStamped, lambda m: m.twist.linear.x),
            ('/cmd_vel_safe', TwistStamped, lambda m: m.twist.linear.x),
        ):
            publisher = self.create_publisher(Float32, topic + '_kmh', 10)
            self._subscriptions.append(self.create_subscription(
                msg_type, topic,
                lambda msg, pub=publisher, get=extract: self._publish(pub, get(msg)),
                qos_profile_sensor_data,
            ))

    @staticmethod
    def _publish(publisher, speed_m_s):
        message = Float32()
        message.data = float(speed_m_s) * 3.6
        publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedDisplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
