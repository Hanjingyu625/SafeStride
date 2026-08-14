
import json, time
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from torchvision import transforms

MODEL_PATH = "road_surface_public_mix_torchscript.pt"
CLASS_PATH = "target_classes.json"
POLICY_PATH = "assist_policy.json"
WEAK_TOP_CROP = 0.0

target_classes = json.loads(Path(CLASS_PATH).read_text(encoding="utf-8"))
assist_policy = json.loads(Path(POLICY_PATH).read_text(encoding="utf-8"))
model = torch.jit.load(MODEL_PATH, map_location="cpu").eval()

tfm = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

def frame_to_input(frame_bgr):
    h, w = frame_bgr.shape[:2]
    frame_bgr = frame_bgr[int(h * WEAK_TOP_CROP):h, :]
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(rgb)
    return tfm(img).unsqueeze(0)

def probs_to_control(probs):
    label = target_classes[int(np.argmax(probs))]
    confidence = float(np.max(probs))
    assist = 0.0
    torque_limit = 0.0
    for i, p in enumerate(probs):
        item = assist_policy[target_classes[i]]
        assist += float(p) * item["assist"]
        torque_limit += float(p) * item["torque_limit"]
    if confidence < assist_policy["__smoothing__"]["min_confidence"]:
        assist += assist_policy["__smoothing__"]["uncertain_assist_bonus"]
    assist = min(assist, assist_policy["__smoothing__"]["max_assist"])
    return label, confidence, assist, torque_limit

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ema_probs = None
alpha = assist_policy["__smoothing__"]["probability_ema_alpha"]

while True:
    ok, frame = cap.read()
    if not ok:
        continue
    x = frame_to_input(frame)
    with torch.no_grad():
        probs = torch.softmax(model(x), dim=1).squeeze(0).numpy()
    ema_probs = probs if ema_probs is None else alpha * ema_probs + (1 - alpha) * probs
    label, confidence, assist, torque_limit = probs_to_control(ema_probs)
    print(f"ASSIST,{assist:.2f},LIMIT,{torque_limit:.2f},LABEL,{label},CONF,{confidence:.2f}")
    time.sleep(0.1)
