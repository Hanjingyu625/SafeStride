"""ROS message/bridge integration; no physical serial port is opened."""
import unittest

try:
    import rclpy
    from rclpy.parameter import Parameter
    from safestride_bridge.terrain_bridge_node import TerrainBridgeNode
    from safestride_interfaces.msg import WalkerStatus
except ImportError:
    rclpy = None

from safestride_bridge.hmi_model import (
    CAPABILITY, FORMAT, STATUS_FORMAT, STATUS_PACKET_TYPE,
)
from safestride_bridge.protocol import Frame


@unittest.skipIf(rclpy is None, 'requires built ROS workspace')
class HmiRosTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = TerrainBridgeNode()
        self.node._session_started = True
        self.node._session_id = 123
        self.node._capabilities = CAPABILITY
        self.node._link_ok = lambda *args: True

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_actual_ros_messages_to_serial_snapshot(self):
        message = WalkerStatus()
        message.state = WalkerStatus.STATE_ARMED
        message.link_ok = message.armed = message.deadman = True
        message.speed_valid = True
        message.measured_speed_kmh = 1.25
        self.node._hmi.model.update('walker', message, self.node._now())
        frames = []
        self.node._write_frame = lambda frame: frames.append(frame)
        self.node._hmi.send()
        self.assertEqual(len(frames), 1)
        words = FORMAT.unpack(frames[0].payload)
        self.assertEqual(words[3], 125)
        self.assertEqual(words[15], 3)
        self.node.set_parameters([Parameter('hmi.enabled', value=False)])
        self.node._hmi.send()
        self.assertEqual(len(frames), 1)

    def test_ack_session_validation_and_diagnostics(self):
        now = self.node._now()
        frame = Frame(packet_type=STATUS_PACKET_TYPE, sequence=1, session_id=999,
                      timestamp_ms=0, payload=STATUS_FORMAT.pack(3,1,0,3,0))
        self.node._handle_frame(frame, now)
        self.assertEqual(self.node._session_errors, 1)
        self.assertIsNone(self.node._hmi.last_status)
        frame = Frame(packet_type=STATUS_PACKET_TYPE, sequence=1, session_id=123,
                      timestamp_ms=0, payload=STATUS_FORMAT.pack(3,1,0,3,0))
        self.node._handle_frame(frame, now)
        self.assertEqual(self.node._hmi.last_status, (1,0,3,0))
        self.assertIsNone(self.node._last_telemetry_time)  # ACK cannot refresh sensors
        self.node._diagnostic_tick()  # Real DiagnosticArray serialization
        self.node._reset_link()
        self.assertIsNone(self.node._hmi.last_status)


if __name__ == '__main__':
    unittest.main()
