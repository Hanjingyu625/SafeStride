"""Exercise production relay methods without DDS or a second serial owner."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from safestride_bridge.hmi_model import (
    Snapshot, PACKET_TYPE, CAPABILITY, STATUS_PACKET_TYPE, STATUS_FORMAT,
)
from safestride_bridge.protocol import Frame, FrameParser


path = Path(__file__).resolve().parents[1] / 'safestride_bridge/hmi_relay.py'
tree = ast.parse(path.read_text(encoding='utf-8'))
klass = next(n for n in tree.body if isinstance(n, ast.ClassDef))
scope = dict(Snapshot=Snapshot, PACKET_TYPE=PACKET_TYPE, CAPABILITY=CAPABILITY,
             STATUS_FORMAT=STATUS_FORMAT, Frame=Frame, KeyValue=NS,
             DiagnosticStatus=type('DiagnosticStatus', (), dict(OK=0, WARN=1)),
             __name__='safestride_bridge.hmi_relay', __package__='safestride_bridge',
             WalkerStatus=NS, HandlePressure=NS, CrosswalkStatus=NS,
             TerrainStatus=NS, SurfaceCondition=NS,
             qos_profile_sensor_data=object())
exec(compile(ast.Module(body=[klass], type_ignores=[]), str(path), 'exec'), scope)
Relay = scope['HmiRelay']


class HmiRelayTests(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.frames = []
        self.time = 10.
        self.linked = True
        self.node = NS(
            declare_parameter=lambda k,v: self.params.setdefault(k,v),
            get_parameter=lambda k: NS(value=self.params[k]),
            create_subscription=lambda *args: args,
            create_timer=lambda *args: args,
            _now=lambda: self.time,
            _link_ok=lambda *args: self.linked,
            _session_started=True, _session_id=123, _capabilities=CAPABILITY,
            _topic_status='/terrain/custom_status',
            _next_sequence=lambda: 10,
            _write_frame=lambda frame: self.frames.append(frame),
        )
        self.relay = Relay(self.node)

    def test_existing_owner_sends_versioned_snapshot(self):
        self.relay.send()
        self.assertEqual(len(self.frames), 1)
        wire = self.frames[0].encode()
        decoded = FrameParser().feed(wire)[0]
        self.assertEqual(decoded.packet_type, PACKET_TYPE)
        self.assertEqual(decoded.session_id, 123)
        self.assertEqual(decoded.payload[:2], b'\x03\x00')
        self.assertEqual(self.params['hmi.topics.terrain'], '/terrain/custom_status')
        self.assertEqual(
            self.params['hmi.topics.surface'],
            '/perception/surface_condition',
        )

    def test_old_firmware_and_disconnected_links_receive_nothing(self):
        self.node._capabilities = 1 << 10  # prototype v1
        self.relay.send()
        self.node._capabilities = CAPABILITY
        self.linked = False
        self.relay.send()
        self.linked = True
        self.node._session_started = False
        self.relay.send()
        self.node._session_started = True
        self.params['hmi.enabled'] = False
        self.relay.send()
        self.assertEqual(self.frames, [])

    def status(self, payload=None, seq=1):
        return Frame(packet_type=STATUS_PACKET_TYPE, sequence=seq, session_id=123,
                     timestamp_ms=10000, payload=payload or STATUS_FORMAT.pack(3,1,0,5,0))

    def test_ack_health_expires_and_resets(self):
        self.assertTrue(self.relay.accept_status(self.status(), 10))
        self.assertEqual(self.relay.diagnostic(10).level, 0)
        self.assertFalse(self.relay.accept_status(self.status(), 10.1))
        self.assertEqual(self.relay.diagnostic(11.5).level, 1)
        self.relay.reset()
        self.assertEqual(self.relay.diagnostic(10).level, 1)
        self.assertTrue(self.relay.accept_status(self.status(), 10))
        self.assertFalse(self.relay.accept_status(self.status(b'bad', 2), 10))
        self.assertTrue(self.relay.accept_status(
            self.status(STATUS_FORMAT.pack(3,0,2,5,1), 2), 10))
        self.assertIn('ACK missing', self.relay.diagnostic(10).message)


if __name__ == '__main__':
    unittest.main()
