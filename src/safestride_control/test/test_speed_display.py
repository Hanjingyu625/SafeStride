"""Exercise km/h topic routing using the production class with ROS substitutes."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class FakeNode:
    def __init__(self, name):
        self.outputs = {}
        self.inputs = {}

    def create_publisher(self, msg_type, topic, qos):
        messages = self.outputs.setdefault(topic, [])
        return SimpleNamespace(publish=messages.append)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.inputs[topic] = callback
        return callback


class TestSpeedDisplay(unittest.TestCase):
    def test_all_routes_convert_once_and_only_on_input(self):
        path = Path(__file__).parents[1] / 'safestride_control/speed_display_node.py'
        tree = ast.parse(path.read_text())
        tree.body = [node for node in tree.body if isinstance(node, ast.ClassDef)]
        scope = dict(Node=FakeNode, Float32=SimpleNamespace,
                     TwistStamped=SimpleNamespace, Odometry=SimpleNamespace,
                     qos_profile_sensor_data=None)
        exec(compile(tree, str(path), 'exec'), scope)
        node = scope['SpeedDisplay']()
        self.assertEqual(set(node.inputs), {
            '/gps/speed', '/gps/speed_raw', '/odom', '/cmd_vel', '/cmd_vel_safe'})
        for topic, callback in node.inputs.items():
            self.assertEqual(node.outputs[topic + '_kmh'], [])
            for speed in (0.0, 8.0 / 3.6, -1.0):
                twist = SimpleNamespace(linear=SimpleNamespace(x=speed))
                msg = SimpleNamespace(data=speed, twist=(
                    SimpleNamespace(twist=twist) if topic == '/odom' else twist))
                callback(msg)
                self.assertAlmostEqual(node.outputs[topic + '_kmh'][-1].data,
                                       speed * 3.6)
            self.assertEqual(len(node.outputs[topic + '_kmh']), 3)
