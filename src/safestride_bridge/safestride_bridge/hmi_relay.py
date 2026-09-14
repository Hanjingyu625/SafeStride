"""Attach to the EXISTING TerrainBridgeNode; never open a second serial owner."""
from rclpy.qos import qos_profile_sensor_data
from safestride_interfaces.msg import WalkerStatus, HandlePressure, CrosswalkStatus, TerrainStatus
from .protocol import Frame
from .hmi_model import Snapshot, PACKET_TYPE, CAPABILITY
from .hmi_model import STATUS_FORMAT
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class HmiRelay:
    def __init__(self, node):
        self.node = node
        self.last_status = None
        self.last_status_time = float('-inf')
        self.last_status_sequence = None
        node.declare_parameter('hmi.enabled', True)
        node.declare_parameter('hmi.pitch_sign', -1.0)
        node.declare_parameter('hmi.pitch_offset_rad', 0.0)
        self.model = Snapshot(node.get_parameter('hmi.pitch_sign').value,
                              node.get_parameter('hmi.pitch_offset_rad').value)
        self.subscriptions = []
        for key, msg, default in (
            ('walker', WalkerStatus, '/walker/status'),
            ('pressure', HandlePressure, '/handle/pressure'),
            ('crosswalk', CrosswalkStatus, '/crosswalk/status'),
            ('terrain', TerrainStatus, node._topic_status),
        ):
            node.declare_parameter('hmi.topics.'+key, default)
            topic = node.get_parameter('hmi.topics.'+key).value
            self.subscriptions.append(node.create_subscription(
                msg, topic, lambda m, k=key: self.model.update(k, m, node._now()),
                qos_profile_sensor_data))
        self.timer = node.create_timer(.2, self.send)

    def reset(self):
        self.last_status = None
        self.last_status_time = float('-inf')
        self.last_status_sequence = None

    def accept_status(self, frame, now):
        from .protocol import sequence_is_newer
        if len(frame.payload) != STATUS_FORMAT.size:
            return False
        version, linked, exception, acks, errors = STATUS_FORMAT.unpack(frame.payload)
        if version != 2 or linked > 1 or exception > 255:
            return False
        if (self.last_status_sequence is not None and
                not sequence_is_newer(frame.sequence, self.last_status_sequence)):
            return False
        self.last_status_sequence = frame.sequence
        self.last_status = linked, exception, acks, errors
        self.last_status_time = now
        return True

    def diagnostic(self, now):
        n = self.node
        status = DiagnosticStatus()
        status.name = 'SafeStride ezHMI display'
        status.hardware_id = 'terrain/D9-TX/D8-RX'
        status.level = DiagnosticStatus.WARN
        if not n.get_parameter('hmi.enabled').value:
            status.level = DiagnosticStatus.OK
            status.message = 'display relay disabled'
        elif not n._link_ok(now):
            status.message = 'Terrain link unavailable'
        elif not n._capabilities & CAPABILITY:
            status.message = 'Terrain firmware lacks HMI v2; flash updated firmware'
        elif self.last_status is None or now - self.last_status_time >= 1.5:
            status.message = 'waiting for LCD transport status'
        elif not self.last_status[0]:
            status.message = 'LCD ACK missing; check wiring and VisualTFT configuration'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'LCD Modbus ACK active (does not verify screen layout)'
        if self.last_status is not None:
            status.values = [KeyValue(key=k, value=str(v)) for k, v in zip(
                ('lcd_link_ok', 'last_exception', 'ack_count', 'error_count'),
                self.last_status)]
        return status

    def send(self):
        n = self.node
        if not n.get_parameter('hmi.enabled').value or not n._session_started or not n._link_ok():
            return
        if not n._capabilities & CAPABILITY:
            return  # Older terrain firmware never receives HMI packets.
        now = n._now()
        n._write_frame(Frame(packet_type=PACKET_TYPE, sequence=n._next_sequence(),
            session_id=n._session_id, timestamp_ms=int(now*1000)&0xffffffff,
            payload=self.model.pack(now)))
