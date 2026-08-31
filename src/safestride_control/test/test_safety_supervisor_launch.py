"""Launch-level test for TerrainStatus fail-safe motor inhibition."""

import time
import unittest

from geometry_msgs.msg import TwistStamped
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

from safestride_interfaces.msg import TerrainStatus, WalkerStatus


@pytest.mark.launch_test
def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package='safestride_control',
        executable='safety_supervisor',
        name='safety_supervisor_integration',
        output='screen',
        parameters=[{
            'publish_rate': 50.0,
            'diagnostic_rate': 5.0,
            'command_timeout': 1.0,
            'status_timeout': 1.0,
            'max_telemetry_age': 1.0,
            'range_timeout': 1.0,
            'require_range_sensors': True,
            'require_deadman': True,
            'max_linear_acceleration': 10.0,
            'max_linear_deceleration': 10.0,
            'command_topic': '/test/cmd_vel',
            'safe_command_topic': '/test/cmd_vel_safe',
            'status_topic': '/test/walker_status',
            'terrain_status_topic': '/test/terrain_status',
        }],
    )
    return launch.LaunchDescription([
        supervisor,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTerrainFailSafe(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_supervisor_test_client')
        cls.command_pub = cls.node.create_publisher(
            TwistStamped, '/test/cmd_vel', 10
        )
        cls.walker_pub = cls.node.create_publisher(
            WalkerStatus, '/test/walker_status', 10
        )
        cls.terrain_pub = cls.node.create_publisher(
            TerrainStatus, '/test/terrain_status', 10
        )
        cls.outputs = []
        cls.node.create_subscription(
            TwistStamped,
            '/test/cmd_vel_safe',
            lambda message: cls.outputs.append(message.twist.linear.x),
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_inputs(
        self,
        alert: int,
        duration: float = 1.0,
        fault_bits: int = 0,
    ):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            command = TwistStamped()
            command.twist.linear.x = 0.10
            walker = WalkerStatus()
            walker.state = WalkerStatus.STATE_ARMED
            walker.link_ok = True
            walker.armed = True
            walker.deadman = True
            walker.telemetry_age = 0.0
            terrain = TerrainStatus()
            terrain.tof_valid = True
            terrain.tof_alert = alert
            terrain.terrain_hazard = alert in (
                TerrainStatus.TOF_RAISED,
                TerrainStatus.TOF_DROP,
            )
            terrain.mpu_valid = not bool(
                fault_bits & TerrainStatus.FAULT_MPU_INVALID
            )
            terrain.fault_bits = fault_bits
            terrain.telemetry_age = 0.0
            self.command_pub.publish(command)
            self.walker_pub.publish(walker)
            self.terrain_pub.publish(terrain)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_graph(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (
                self.command_pub.get_subscription_count() > 0
                and self.walker_pub.get_subscription_count() > 0
                and self.terrain_pub.get_subscription_count() > 0
                and self.node.count_publishers('/test/cmd_vel_safe') > 0
            ):
                return
        self.fail('safety supervisor ROS graph did not become ready')

    def test_confirmed_drop_stops_output(self):
        self._wait_for_graph()
        self._publish_inputs(
            TerrainStatus.TOF_NORMAL,
            1.5,
            TerrainStatus.FAULT_MPU_INVALID,
        )
        self.assertTrue(any(value > 0.01 for value in self.outputs))
        self.outputs.clear()
        self._publish_inputs(TerrainStatus.TOF_DROP, 1.0)
        self.assertTrue(self.outputs)
        self.assertEqual(self.outputs[-1], 0.0)
