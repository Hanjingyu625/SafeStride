"""Publish fail-safe road-surface classifications from a Pi camera."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from safestride_interfaces.msg import SurfaceCondition

from .surface_policy import speed_scale


MODEL_CLASSIFICATIONS = {
    'smooth_paved': SurfaceCondition.SMOOTH,
    'rough_paved': SurfaceCondition.ROUGH,
    'block_paved': SurfaceCondition.ROUGH,
    'gravel': SurfaceCondition.GRAVEL,
    'mud_dirt': SurfaceCondition.ROUGH,
    'unpaved_mixed': SurfaceCondition.ROUGH,
    'wet_paved': SurfaceCondition.WET,
    'wet_unpaved': SurfaceCondition.WET,
    'snow_ice': SurfaceCondition.WET,
}


def _finite_parameter(
    name: str,
    value: object,
    minimum: float,
    maximum: float,
    *,
    minimum_inclusive: bool = True,
) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be numeric')
    number = float(value)
    lower_ok = number >= minimum if minimum_inclusive else number > minimum
    if not math.isfinite(number) or not lower_ok or number > maximum:
        operator = '>=' if minimum_inclusive else '>'
        raise ValueError(
            f'{name} must be finite and {operator} {minimum} and <= {maximum}'
        )
    return number


class SurfacePerceptionNode(Node):
    """Run TorchScript inference and publish conservative surface limits."""

    def __init__(self) -> None:
        super().__init__('surface_perception')

        self.declare_parameter('camera.index', 0)
        self.declare_parameter('camera.backend', 'v4l2')
        self.declare_parameter('camera.width', 640)
        self.declare_parameter('camera.height', 480)
        self.declare_parameter('camera.reconnect_period_s', 2.0)
        self.declare_parameter('model.path', '')
        self.declare_parameter('model.classes_path', '')
        self.declare_parameter('model.confidence_threshold', 0.65)
        self.declare_parameter('model.ema_alpha', 0.75)
        self.declare_parameter('model.version', '')
        self.declare_parameter('inference_rate_hz', 5.0)
        self.declare_parameter('diagnostic_rate_hz', 1.0)
        self.declare_parameter(
            'surface_topic', '/perception/surface_condition'
        )
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('frame_id', 'camera_link')

        self._camera_index = int(self.get_parameter('camera.index').value)
        if self._camera_index < 0:
            raise ValueError('camera.index must be non-negative')
        self._camera_backend = str(
            self.get_parameter('camera.backend').value
        ).strip().lower()
        if self._camera_backend not in ('auto', 'v4l2'):
            raise ValueError('camera.backend must be auto or v4l2')
        self._camera_width = int(self.get_parameter('camera.width').value)
        self._camera_height = int(self.get_parameter('camera.height').value)
        if self._camera_width <= 0 or self._camera_height <= 0:
            raise ValueError('camera dimensions must be positive')
        self._camera_reconnect_period = _finite_parameter(
            'camera.reconnect_period_s',
            self.get_parameter('camera.reconnect_period_s').value,
            0.0,
            60.0,
            minimum_inclusive=False,
        )
        self._confidence_threshold = _finite_parameter(
            'model.confidence_threshold',
            self.get_parameter('model.confidence_threshold').value,
            0.0,
            1.0,
        )
        self._ema_alpha = _finite_parameter(
            'model.ema_alpha',
            self.get_parameter('model.ema_alpha').value,
            0.0,
            1.0,
        )
        self._inference_rate = _finite_parameter(
            'inference_rate_hz',
            self.get_parameter('inference_rate_hz').value,
            0.0,
            30.0,
            minimum_inclusive=False,
        )
        self._diagnostic_rate = _finite_parameter(
            'diagnostic_rate_hz',
            self.get_parameter('diagnostic_rate_hz').value,
            0.0,
            10.0,
            minimum_inclusive=False,
        )
        self._frame_id = str(self.get_parameter('frame_id').value)

        try:
            import cv2
            import numpy as np
            import torch
        except ImportError as error:
            raise RuntimeError(
                'surface perception requires cv2, numpy and torch; '
                'run scripts/install_perception.sh'
            ) from error
        self._cv2 = cv2
        self._np = np
        self._torch = torch

        model_path = Path(str(self.get_parameter('model.path').value))
        classes_path = Path(
            str(self.get_parameter('model.classes_path').value)
        )
        if not model_path.is_file():
            raise ValueError(f'model.path does not exist: {model_path}')
        if not classes_path.is_file():
            raise ValueError(
                f'model.classes_path does not exist: {classes_path}'
            )

        raw_classes = json.loads(classes_path.read_text(encoding='utf-8'))
        if (
            not isinstance(raw_classes, list)
            or not raw_classes
            or not all(isinstance(item, str) for item in raw_classes)
        ):
            raise ValueError('model.classes_path must contain a string list')
        unsupported = sorted(set(raw_classes) - set(MODEL_CLASSIFICATIONS))
        if unsupported:
            raise ValueError(
                'unsupported model classes: ' + ', '.join(unsupported)
            )
        self._classes: List[str] = raw_classes
        self._model = self._torch.jit.load(
            str(model_path), map_location='cpu'
        ).eval()

        configured_version = str(
            self.get_parameter('model.version').value
        ).strip()
        if configured_version:
            self._model_version = configured_version
        else:
            digest = hashlib.sha256(model_path.read_bytes()).hexdigest()[:12]
            self._model_version = f'{model_path.stem}:{digest}'

        self._surface_publisher = self.create_publisher(
            SurfaceCondition,
            str(self.get_parameter('surface_topic').value),
            qos_profile_sensor_data,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )

        self._camera = None
        self._last_camera_attempt = -math.inf
        self._consecutive_read_failures = 0
        self._ema_probabilities = None
        self._last_label = 'unavailable'
        self._last_confidence = math.nan
        self._last_scale = 0.0
        self._last_valid = False
        self._last_error = 'camera_not_open'
        self._last_diagnostic_time = -math.inf

        self._timer = self.create_timer(
            1.0 / self._inference_rate, self._timer_callback
        )
        self.get_logger().info(
            'Surface perception ready at %.1f Hz using model %s'
            % (self._inference_rate, self._model_version)
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _open_camera(self, now: float) -> bool:
        if now - self._last_camera_attempt < self._camera_reconnect_period:
            return False
        self._last_camera_attempt = now
        backend = (
            self._cv2.CAP_V4L2
            if self._camera_backend == 'v4l2'
            else self._cv2.CAP_ANY
        )
        camera = self._cv2.VideoCapture(self._camera_index, backend)
        camera.set(self._cv2.CAP_PROP_FRAME_WIDTH, self._camera_width)
        camera.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self._camera_height)
        camera.set(self._cv2.CAP_PROP_BUFFERSIZE, 1)
        if not camera.isOpened():
            camera.release()
            self._last_error = 'camera_open_failed'
            self.get_logger().warning(
                'Cannot open camera index %d with backend %s'
                % (self._camera_index, self._camera_backend),
                throttle_duration_sec=5.0,
            )
            return False
        self._camera = camera
        self._consecutive_read_failures = 0
        self._ema_probabilities = None
        self.get_logger().info(
            'Opened camera index %d with backend %s'
            % (self._camera_index, self._camera_backend)
        )
        return True

    def _close_camera(self) -> None:
        if self._camera is not None:
            self._camera.release()
            self._camera = None

    def _frame_tensor(self, frame):
        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        resized = self._cv2.resize(
            rgb, (224, 224), interpolation=self._cv2.INTER_AREA
        )
        values = resized.astype(self._np.float32) / 255.0
        mean = self._np.asarray(
            [0.485, 0.456, 0.406], dtype=self._np.float32
        )
        std = self._np.asarray(
            [0.229, 0.224, 0.225], dtype=self._np.float32
        )
        values = (values - mean) / std
        channels_first = self._np.ascontiguousarray(
            self._np.transpose(values, (2, 0, 1))
        )
        return self._torch.from_numpy(channels_first).unsqueeze(0)

    def _infer(self, frame) -> Dict[str, object]:
        tensor = self._frame_tensor(frame)
        with self._torch.inference_mode():
            output = self._model(tensor)
        if isinstance(output, (tuple, list)):
            output = output[0]
        logits = output.squeeze()
        if logits.ndim != 1 or int(logits.numel()) != len(self._classes):
            raise ValueError(
                'model output size does not match target_classes.json'
            )
        probabilities = (
            self._torch.softmax(logits, dim=0).detach().cpu().numpy()
        )
        if self._ema_probabilities is None:
            self._ema_probabilities = probabilities
        else:
            self._ema_probabilities = (
                self._ema_alpha * self._ema_probabilities
                + (1.0 - self._ema_alpha) * probabilities
            )
        index = int(self._np.argmax(self._ema_probabilities))
        label = self._classes[index]
        confidence = float(self._ema_probabilities[index])
        valid = (
            math.isfinite(confidence)
            and confidence >= self._confidence_threshold
            and label in MODEL_CLASSIFICATIONS
        )
        scale = (
            speed_scale(label, confidence, self._confidence_threshold)
            if valid
            else 0.0
        )
        return {
            'label': label,
            'confidence': confidence,
            'classification': MODEL_CLASSIFICATIONS.get(
                label, SurfaceCondition.UNKNOWN
            ),
            'scale': scale,
            'valid': valid,
        }

    def _publish_surface(
        self,
        *,
        classification: int = SurfaceCondition.UNKNOWN,
        confidence: float = 0.0,
        scale: float = 0.0,
        valid: bool = False,
    ) -> None:
        message = SurfaceCondition()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.classification = classification
        message.confidence = confidence
        message.recommended_speed_scale = scale
        message.valid = valid
        message.model_version = self._model_version
        self._surface_publisher.publish(message)

    def _timer_callback(self) -> None:
        now = self._now_seconds()
        if self._camera is None and not self._open_camera(now):
            self._last_valid = False
            self._publish_surface()
            self._maybe_publish_diagnostics(now)
            return

        ok, frame = self._camera.read()
        if not ok or frame is None:
            self._consecutive_read_failures += 1
            self._last_error = 'camera_read_failed'
            self._last_valid = False
            self._publish_surface()
            if self._consecutive_read_failures >= 3:
                self._close_camera()
            self._maybe_publish_diagnostics(now)
            return

        self._consecutive_read_failures = 0
        try:
            result = self._infer(frame)
        except Exception as error:
            self._last_error = f'inference_failed: {error}'
            self._last_valid = False
            self._publish_surface()
            self.get_logger().error(
                self._last_error, throttle_duration_sec=5.0
            )
            self._maybe_publish_diagnostics(now)
            return

        self._last_label = str(result['label'])
        self._last_confidence = float(result['confidence'])
        self._last_scale = float(result['scale'])
        self._last_valid = bool(result['valid'])
        self._last_error = '' if self._last_valid else 'low_confidence'
        self._publish_surface(
            classification=int(result['classification']),
            confidence=self._last_confidence,
            scale=self._last_scale,
            valid=self._last_valid,
        )
        self._maybe_publish_diagnostics(now)

    def _maybe_publish_diagnostics(self, now: float) -> None:
        if now - self._last_diagnostic_time < 1.0 / self._diagnostic_rate:
            return
        status = DiagnosticStatus()
        status.name = 'SafeStride/Surface Perception'
        status.hardware_id = f'camera-{self._camera_index}'
        if self._last_valid:
            status.level = DiagnosticStatus.OK
            status.message = 'surface classification valid'
        elif self._camera is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_error or 'camera unavailable'
        else:
            status.level = DiagnosticStatus.WARN
            status.message = self._last_error or 'classification invalid'
        status.values = [
            KeyValue(key='label', value=self._last_label),
            KeyValue(
                key='confidence',
                value=(
                    'unavailable'
                    if not math.isfinite(self._last_confidence)
                    else '%.3f' % self._last_confidence
                ),
            ),
            KeyValue(key='speed_scale', value='%.3f' % self._last_scale),
            KeyValue(key='valid', value=str(self._last_valid).lower()),
            KeyValue(key='model_version', value=self._model_version),
            KeyValue(key='camera_backend', value=self._camera_backend),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostic_publisher.publish(array)
        self._last_diagnostic_time = now

    def destroy_node(self):
        self._close_camera()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SurfacePerceptionNode] = None
    try:
        node = SurfacePerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
