
"""Standalone Pi camera check; production uses the ROS perception node."""

import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch


BASE_PATH = Path(__file__).resolve().parent
MODEL_PATH = BASE_PATH / 'road_surface_public_mix_torchscript.pt'
CLASS_PATH = BASE_PATH / 'target_classes.json'
POLICY_PATH = BASE_PATH / 'assist_policy.json'
WEAK_TOP_CROP = 0.0

target_classes = json.loads(CLASS_PATH.read_text(encoding='utf-8'))
assist_policy = json.loads(POLICY_PATH.read_text(encoding='utf-8'))
model = torch.jit.load(str(MODEL_PATH), map_location='cpu').eval()


def frame_to_input(frame_bgr):
    height = frame_bgr.shape[0]
    frame_bgr = frame_bgr[int(height * WEAK_TOP_CROP):height, :]
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_AREA)
    values = resized.astype(np.float32) / 255.0
    mean = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
    values = (values - mean) / std
    channels_first = np.ascontiguousarray(values.transpose(2, 0, 1))
    return torch.from_numpy(channels_first).unsqueeze(0)


def probs_to_control(probs):
    label = target_classes[int(np.argmax(probs))]
    confidence = float(np.max(probs))
    assist = 0.0
    torque_limit = 0.0
    for index, probability in enumerate(probs):
        item = assist_policy[target_classes[index]]
        assist += float(probability) * item['assist']
        torque_limit += float(probability) * item['torque_limit']
    smoothing = assist_policy['__smoothing__']
    if confidence < smoothing['min_confidence']:
        assist += smoothing['uncertain_assist_bonus']
    assist = min(assist, smoothing['max_assist'])
    return label, confidence, assist, torque_limit


cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    raise RuntimeError('cannot open Pi camera index 0 with V4L2')

ema_probs = None
alpha = assist_policy['__smoothing__']['probability_ema_alpha']

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError('Pi camera frame read failed')
        inputs = frame_to_input(frame)
        with torch.inference_mode():
            probs = torch.softmax(model(inputs), dim=1).squeeze(0).numpy()
        ema_probs = (
            probs
            if ema_probs is None
            else alpha * ema_probs + (1.0 - alpha) * probs
        )
        label, confidence, assist, torque_limit = probs_to_control(ema_probs)
        print(
            f'ASSIST,{assist:.2f},LIMIT,{torque_limit:.2f},'
            f'LABEL,{label},CONF,{confidence:.2f}'
        )
        time.sleep(0.1)
finally:
    cap.release()
