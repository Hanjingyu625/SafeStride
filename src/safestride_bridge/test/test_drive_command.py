"""Exercise the actual bridge callbacks without serial hardware or DDS."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest

from safestride_bridge.protocol import CommandPayload, PacketType, PayloadDecodeError


path = Path(__file__).resolve().parents[1] / 'safestride_bridge/serial_bridge_node.py'
tree = ast.parse(path.read_text(encoding='utf-8'))
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SerialBridgeNode')
node.bases = []
node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in
             ('_on_cmd_vel', '_command_tick', '_send_command')]
scope = dict(math=math, DriveCommand=NS, CommandPayload=CommandPayload, PacketType=PacketType)
exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), scope)
Bridge = scope['SerialBridgeNode']


class TestDriveCommand(unittest.TestCase):
    def setUp(self):
        self.b = Bridge()
        b = self.b
        b._lock = threading.RLock()
        b._command_timeout = 0.5
        b._command_ttl_ms = 200
        b._max_abs_angular_z = 0.0
        b._wheel_radius = 0.115
        b._max_wheel_speed = 3.0
        b._deadman_direct_drive = False
        b._level_enable_blocked = False
        b._session_started = True
        b._serial_connected = lambda: True
        b._last_telemetry = object()
        b._link_ok = lambda now: True
        b._remote_allows_deadman_ramp = lambda telemetry: False
        b._remote_allows_enable = lambda telemetry: True
        b._now_monotonic = lambda: 10.0
        b.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
        b.get_logger = lambda: NS(error=lambda *a: None, warning=lambda *a: None)
        self.packets = []
        b._make_frame = lambda kind, payload: payload
        b._write_frame = lambda payload: self.packets.append(CommandPayload.unpack(payload)) or True

    def message(self, target=0.08, ff=8, mode=0, stamp=10, cap=100):
        return NS(target_linear_m_s=target, slope_ff_pwm=ff, drive_pwm_cap=cap,
                  mode=mode, header=NS(stamp=NS(sec=stamp, nanosec=0)))

    def test_atomic_drive_then_brake_then_drive(self):
        self.b._on_cmd_vel(self.message())
        self.b._command_tick()
        p = self.packets[-1]
        self.assertEqual((p.target_mrad_s, p.slope_ff_pwm, p.mode, p.enable), (696, 8, 0, 1))
        self.b._on_cmd_vel(self.message(0, 0, 1))
        self.b._command_tick()
        self.assertEqual((self.packets[-1].mode, self.packets[-1].enable), (1, 1))
        self.b._on_cmd_vel(self.message(ff=-18))
        self.b._command_tick()
        self.assertEqual((self.packets[-1].mode, self.packets[-1].slope_ff_pwm), (0, -18))

    def test_stale_future_invalid_messages_cannot_refresh_command(self):
        for msg in (self.message(stamp=9), self.message(stamp=11),
                    self.message(target=math.nan), self.message(ff=31),
                    self.message(mode=2), self.message(cap=101), self.message(mode=1)):
            self.b._on_cmd_vel(self.message())
            self.b._on_cmd_vel(msg)
            self.assertIsNone(self.b._last_command_time)
            self.assertEqual(self.packets[-1].enable, 0)
            self.b._command_tick()
            self.assertEqual(self.packets[-1].enable, 0)

    def test_timeout_and_deadman_release_drop_slope_ff(self):
        self.b._on_cmd_vel(self.message())
        self.b._now_monotonic = lambda: 11.0
        self.b._command_tick()
        self.assertEqual((self.packets[-1].enable, self.packets[-1].slope_ff_pwm), (0, 0))
        self.b._remote_allows_deadman_ramp = lambda telemetry: True
        self.b._command_tick()
        self.assertEqual((self.packets[-1].target_mrad_s, self.packets[-1].slope_ff_pwm), (0, 0))

    def test_v4_and_invalid_v5_payloads_rejected(self):
        import struct
        for raw in (struct.pack('<iHBB', 0, 200, 1, 0),
                    struct.pack('<iHBBhBB', 0, 200, 1, 0, -61, 100, 0),
                    struct.pack('<iHBBhBB', 0, 200, 1, 0, 0, 101, 0),
                    struct.pack('<iHBBhBB', 0, 200, 1, 0, 0, 100, 2)):
            with self.assertRaises(PayloadDecodeError):
                CommandPayload.unpack(raw)


if __name__ == '__main__':
    unittest.main()
