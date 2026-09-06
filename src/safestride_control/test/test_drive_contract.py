"""Execute production node methods with in-memory ROS transport substitutes.

These tests cover command decisions and serialization, not DDS or ROS launch.
"""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import typing
import unittest

from safestride_control.safety_logic import (
    SlopeSpeedPolicy, SlopeBrakePolicy, slope_feedforward_pwm,
    combine_speed_scales, finite_parameter,
)

ROOT = Path(__file__).resolve().parents[3]


class Message(NS):
    DRIVE = 0
    BRAKE = 1
    UNKNOWN = 0
    FAULT_MPU_INVALID = 2

    def __init__(self, **kwargs):
        super().__init__(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=''),
                         twist=NS(linear=NS(x=0.0), angular=NS(z=0.0)))
        self.__dict__.update(kwargs)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self, *args):
        self.params = {}
        self.now = 10.0

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.now * 1e9),
                                to_msg=lambda: NS(sec=int(self.now), nanosec=0)))

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_logger(self):
        return NS(info=lambda *a: None, warning=lambda *a: None,
                  error=lambda *a: None)


def production_class(relative, name):
    tree = ast.parse((ROOT / relative).read_text(encoding='utf-8'))
    # Keep the actual class and helper function bodies; replace only imports.
    tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
                 and not (isinstance(n, ast.FunctionDef) and n.name == 'main')]
    scope = dict(vars(typing), math=math, Node=FakeNode, threading=threading,
                 DriveCommand=Message, TwistStamped=Message, Range=Message,
                 SurfaceCondition=Message, TerrainStatus=Message, WalkerStatus=Message,
                 DiagnosticArray=Message, DiagnosticStatus=Message, KeyValue=Message,
                 SlopeSpeedPolicy=SlopeSpeedPolicy, SlopeBrakePolicy=SlopeBrakePolicy,
                 slope_feedforward_pwm=slope_feedforward_pwm,
                 combine_speed_scales=combine_speed_scales, finite_parameter=finite_parameter,
                 QoSProfile=lambda **kw: None, qos_profile_sensor_data=None,
                 HistoryPolicy=NS(KEEP_LAST=0), ReliabilityPolicy=NS(RELIABLE=0),
                 DurabilityPolicy=NS(VOLATILE=0))
    exec(compile(tree, str(ROOT / relative), 'exec'), scope)
    return scope[name]


Supervisor = production_class(
    'src/safestride_control/safestride_control/safety_supervisor_node.py', 'SafetySupervisor')


class TestSupervisedDrive(unittest.TestCase):
    def setUp(self):
        self.node = Supervisor()
        n = self.node
        n._command_reasons = lambda now: []
        n._status_reasons = lambda now: []
        n._terrain_reasons = lambda now: []
        n._range_state = lambda now: ([], {}, {'left': 1.0, 'right': 1.0})
        n._publish_diagnostics = lambda *args: None
        n._last_command = Message()
        n._last_command.twist.linear.x = 0.08

    def tick(self, pitch, age=0.0, valid=True, dt=0.05):
        n = self.node
        n.now += dt
        n._last_terrain_time = n._now_seconds()
        n._last_terrain = NS(pitch_rad=math.radians(pitch), telemetry_age=age,
                             mpu_valid=valid, fault_bits=0)
        n._timer_callback()
        return n._drive_publisher.messages[-1]

    def test_full_drive_brake_auto_resume_and_atomic_fields(self):
        for _ in range(20):
            msg = self.tick(0)
        self.assertAlmostEqual(msg.target_linear_m_s, 0.08)
        self.assertEqual((msg.mode, msg.slope_ff_pwm), (0, 0))
        for _ in range(20):
            msg = self.tick(5.5)
        self.assertAlmostEqual(msg.target_linear_m_s, 0.08)
        self.assertEqual(msg.slope_ff_pwm, 10)
        msg = self.tick(-11)
        self.assertEqual((msg.mode, msg.target_linear_m_s, msg.slope_ff_pwm), (1, 0.0, 0))
        for _ in range(20):
            self.assertEqual(self.tick(-8).mode, 1)
        for _ in range(30):
            msg = self.tick(0)
        self.assertEqual(msg.mode, 0)
        self.assertGreater(msg.target_linear_m_s, 0.0)
        self.assertFalse(self.node._command_output_suppressed)

    def test_invalid_imu_keeps_streaming_brake_then_recovers(self):
        for bad_age in (-1.0, math.nan, math.inf, 0.4):
            msg = self.tick(-6, age=bad_age)
            self.assertEqual((msg.mode, msg.slope_ff_pwm), (1, 0))
            self.assertFalse(self.node._command_output_suppressed)
        self.assertEqual(self.tick(0, valid=False).mode, 1)
        for _ in range(30):
            msg = self.tick(0)
        self.assertEqual(msg.mode, 0)

    def test_downhill_continuous_scale_and_surface_disabled(self):
        # Even a live surface result cannot modify this deployment's speed.
        self.node._last_surface = NS(valid=True, recommended_speed_scale=0.0)
        for _ in range(20):
            msg = self.tick(-6)
        self.assertAlmostEqual(msg.target_linear_m_s, 0.08 * 0.76)
        self.assertEqual(msg.slope_ff_pwm, -18)
        for _ in range(20):
            msg = self.tick(-8)
        self.assertAlmostEqual(msg.target_linear_m_s, 0.08 * 0.6)
        self.assertEqual(msg.slope_ff_pwm, -30)

    def test_existing_fault_still_suppresses_stream(self):
        self.node._status_reasons = lambda now: ['mcu_fault']
        msg = self.tick(0)
        self.assertEqual((msg.mode, msg.target_linear_m_s), (1, 0.0))
        count = len(self.node._drive_publisher.messages)
        self.tick(0)
        self.assertEqual(len(self.node._drive_publisher.messages), count)

    def test_freshness_includes_age_since_ros_delivery(self):
        self.tick(0, age=0.2)
        self.node.now += 0.2
        scale, state, pitch = self.node._slope_state(self.node.now)
        self.assertEqual((scale, state), (0.0, 'unavailable'))
        self.assertTrue(math.isnan(pitch))

    def test_reverse_has_no_forward_slope_assist(self):
        self.node._last_command.twist.linear.x = -0.05
        for _ in range(20):
            msg = self.tick(8)
        self.assertEqual(msg.slope_ff_pwm, 0)
        self.assertAlmostEqual(msg.target_linear_m_s, -0.05)

    def test_invalid_brake_parameters(self):
        for args in ((math.nan, 7, 0.5), (7, 7, 0.5), (10, 7, -1)):
            with self.assertRaises(ValueError):
                SlopeBrakePolicy(*args)


if __name__ == '__main__':
    unittest.main()
