"""Attach to the EXISTING TerrainBridgeNode; never open a second serial owner."""
from rclpy.qos import qos_profile_sensor_data
from safestride_interfaces.msg import WalkerStatus, HandlePressure, CrosswalkStatus, TerrainStatus
from .protocol import Frame
from .hmi_model import Snapshot, PACKET_TYPE, CAPABILITY


class HmiRelay:
    def __init__(self, node):
        self.node = node
        node.declare_parameter('hmi.enabled', True)
        node.declare_parameter('hmi.pitch_sign', 1.0)
        node.declare_parameter('hmi.pitch_offset_rad', 0.0)
        self.model = Snapshot(node.get_parameter('hmi.pitch_sign').value,
                              node.get_parameter('hmi.pitch_offset_rad').value)
        self.subscriptions = []
        for key, msg, default in (
            ('walker', WalkerStatus, '/walker/status'),
            ('pressure', HandlePressure, '/handle/pressure'),
            ('crosswalk', CrosswalkStatus, '/crosswalk/status'),
            ('terrain', TerrainStatus, '/terrain/status'),
        ):
            node.declare_parameter('hmi.topics.'+key, default)
            topic = node.get_parameter('hmi.topics.'+key).value
            self.subscriptions.append(node.create_subscription(
                msg, topic, lambda m, k=key: self.model.update(k, m, node._now()),
                qos_profile_sensor_data))
        self.timer = node.create_timer(.2, self.send)

    def send(self):
        n = self.node
        if not n.get_parameter('hmi.enabled').value or not n._session_started or not n._link_ok():
            return
        if not n._capabilities & CAPABILITY:
            return  # Unmodified terrain firmware never receives experimental packets.
        now = n._now()
        n._write_frame(Frame(packet_type=PACKET_TYPE, sequence=n._next_sequence(),
            session_id=n._session_id, timestamp_ms=int(now*1000)&0xffffffff,
            payload=self.model.pack(now)))
