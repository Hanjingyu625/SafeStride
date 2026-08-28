#!/usr/bin/env python3
"""Build the standalone Colab notebook from the checked-in training tools."""

from __future__ import annotations

import base64
import gzip
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NOTEBOOK_PATH = ROOT / "notebooks" / "road_surface_training_colab.ipynb"
EMBEDDED_FILES = (
    ROOT / "tools" / "road_surface_labels.py",
    ROOT / "tools" / "train_road_surface.py",
)


def lines(text: str) -> list[str]:
    return text.splitlines(keepends=True)


def markdown(text: str) -> dict[str, object]:
    return {"cell_type": "markdown", "metadata": {}, "source": lines(text)}


def code(text: str) -> dict[str, object]:
    return {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": lines(text),
    }


def embedded_tools_cell() -> str:
    encoded = {
        path.name: base64.b64encode(
            gzip.compress(path.read_bytes(), mtime=0)
        ).decode("ascii")
        for path in EMBEDDED_FILES
    }
    payload = json.dumps(encoded, indent=4, sort_keys=True)
    return f"""from pathlib import Path
import base64
import gzip
import subprocess

# The standalone notebook writes the exact checked-in tools to Colab.
REPO_DIR = Path('/content/SafeStrideTraining')
TOOLS_DIR = REPO_DIR / 'tools'
TOOLS_DIR.mkdir(parents=True, exist_ok=True)
EMBEDDED_TOOLS = {payload}
for filename, encoded in EMBEDDED_TOOLS.items():
    source = gzip.decompress(base64.b64decode(encoded))
    (TOOLS_DIR / filename).write_bytes(source)
print('Prepared standalone training tools in', TOOLS_DIR)
"""


def build_notebook() -> dict[str, object]:
    cells = [
        markdown(
            """# SafeStride road-surface training

This notebook rebuilds the ROS-compatible nine-class road-surface model. It uses grouped train/validation/test splits, fine-tunes MobileNetV3-Small for Raspberry Pi deployment, evaluates every class, and approves a production artifact only when the held-out test gate passes.

The executable training tools are embedded for standalone use and are also displayed below. Dataset preparation and epoch checkpoints persist in Google Drive, so a compatible interrupted run resumes instead of restarting. Select a GPU runtime before running all cells.
"""
        ),
        code(
            """!nvidia-smi
!pip -q install -U scikit-learn pandas pillow requests tqdm huggingface_hub

import torch
assert torch.cuda.is_available(), 'Select Runtime > Change runtime type > GPU, then reconnect.'
try:
    from torch.ao.quantization.quantize_fx import convert_fx, prepare_fx
except ImportError as error:
    raise RuntimeError('This export pipeline requires PyTorch FX static quantization support.') from error
print('torch:', torch.__version__)
print('GPU:', torch.cuda.get_device_name(0))
"""
        ),
        code(embedded_tools_cell()),
        markdown(
            """## Inspect the exact implementation

The short training cell later in the notebook is only a process launcher. These cells display the exact source that it executes: dataset and loaders, optimizer/training loop, and post-training static INT8 calibration/conversion.
"""
        ),
        code(
            """import importlib
import inspect
import sys
from IPython.display import Code, Markdown, display

sys.path.insert(0, str(TOOLS_DIR))
sys.modules.pop('train_road_surface', None)
trainer = importlib.import_module('train_road_surface')

for name in ('RoadSurfaceDataset', 'build_loaders', 'build_calibration_loader'):
    display(Markdown(f'### `{name}`'))
    display(Code(inspect.getsource(getattr(trainer, name)), language='python'))
"""
        ),
        code(
            """for name in ('build_model', 'run_epoch', 'train_candidate', 'choose_candidate'):
    display(Markdown(f'### `{name}`'))
    display(Code(inspect.getsource(getattr(trainer, name)), language='python'))
"""
        ),
        code(
            """for name in ('quantize_model', 'passes_export_gate'):
    display(Markdown(f'### `{name}`'))
    display(Code(inspect.getsource(getattr(trainer, name)), language='python'))
"""
        ),
        markdown(
            """## What optimization is actually applied

- Training is FP32 transfer learning with AMP on the GPU.
- Quantization is FX post-training static quantization for the QNNPACK ARM backend. Weights use qint8 and activations use quint8.
- Calibration uses a deterministic, class-balanced subset of the train split with resize/normalize only. Validation and test images are never calibration inputs.
- INT8 is deployed only if validation macro-F1 drops by at most 0.015 and worst per-class recall drops by at most 0.05.
- Pruning is not applied. Unstructured zeroing does not automatically accelerate dense Raspberry Pi kernels, so the pipeline records `pruning: none` rather than claiming an unsupported speedup.
"""
        ),
        code(
            """import os
from google.colab import drive

try:
    from google.colab import userdata
    hf_token = userdata.get('HF_TOKEN')
except Exception:
    hf_token = None
if hf_token:
    os.environ['HF_TOKEN'] = hf_token
    print('Hugging Face authentication enabled.')
else:
    print('Using public Hugging Face access. Optional: add HF_TOKEN in Colab Secrets for higher rate limits.')

drive.mount('/content/drive')
WORK_DIR = Path('/content/drive/MyDrive/SafeStride/road_surface_training_v2')
RUN_DIR = Path('/content/drive/MyDrive/SafeStride/road_surface_mobilenet_v3_small_v1')
EXPORT_DIR = RUN_DIR / 'export'
CHECKPOINT_DIR = RUN_DIR / 'checkpoints'
WORK_DIR.mkdir(parents=True, exist_ok=True)
EXPORT_DIR.mkdir(parents=True, exist_ok=True)
CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)

# Reuse the completed manifest from the earlier three-model notebook once.
PREPARED_MANIFEST = WORK_DIR / 'prepared' / 'dataset_manifest.csv'
LEGACY_EXPORT_ROOT = Path('/content/drive/MyDrive/SafeStride/road_surface_exports_v2')
legacy_manifests = (
    sorted(LEGACY_EXPORT_ROOT.glob('*/dataset_manifest.csv'), key=lambda path: path.stat().st_mtime)
    if LEGACY_EXPORT_ROOT.is_dir()
    else []
)
REUSE_MANIFEST = None if PREPARED_MANIFEST.is_file() else (legacy_manifests[-1] if legacy_manifests else None)
print('cache:', WORK_DIR)
print('exports:', EXPORT_DIR)
print('checkpoints:', CHECKPOINT_DIR)
if REUSE_MANIFEST:
    print('reusing earlier prepared manifest:', REUSE_MANIFEST)
"""
        ),
        markdown(
            """## Train, validate, calibrate, and export

The default run keeps all nine ROS classes. Training requires at least 60 valid images per class and warns below the recommended 250. Validation and test each require 10 independent examples per class. The final held-out test gate requires macro F1 >= 0.75 and recall >= 0.55 for every class.

MobileNetV3-Small is the only trained backbone in this practical Raspberry Pi run. The maximum is 15 fine-tuning epochs, validation macro-F1 controls LR decay and four unimproved fine-tuning epochs trigger early stopping. A checkpoint is atomically written to Drive after every epoch.

RSCD itself has no `block_paved` or `unpaved_mixed` label, so those two RSCD warnings are expected; the other public sources supply those classes. The prepared dataset manifest is cached and reused after its first successful preparation.
"""
        ),
        code(
            """print('Starting dataset preparation and training.', flush=True)
command = [
    sys.executable, '-u', str(TOOLS_DIR / 'train_road_surface.py'),
    '--work-dir', str(WORK_DIR),
    '--export-dir', str(EXPORT_DIR),
    '--checkpoint-dir', str(CHECKPOINT_DIR),
    '--resume',
    '--models', 'mobilenet_v3_small',
    '--batch-size', '128',
    '--finetune-epochs', '15',
    '--early-stop-patience', '4',
    '--rscd-download-workers', '4',
    '--quantize-int8',
]
if REUSE_MANIFEST is not None:
    command.extend(['--dataset-manifest', str(REUSE_MANIFEST)])
print('command:', ' '.join(command), flush=True)
process = subprocess.Popen(
    command,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
)
assert process.stdout is not None
for line in process.stdout:
    print(line, end='', flush=True)
return_code = process.wait()
if return_code != 0:
    raise subprocess.CalledProcessError(return_code, command)
"""
        ),
        code(
            """import json

manifest = json.loads((EXPORT_DIR / 'model_manifest.json').read_text(encoding='utf-8'))
print(json.dumps({
    'model_name': manifest['model_name'],
    'deployment_approved': manifest['deployment_approved'],
    'deployment_gate_reasons': manifest['deployment_gate_reasons'],
    'quantization': manifest['quantization'],
    'quantization_report': manifest['quantization_report'],
    'pruning': manifest['pruning'],
    'size_mb': round(manifest['artifact']['size_bytes'] / 1024 / 1024, 2),
    'validation_macro_f1': manifest['metrics']['validation']['macro_f1'],
    'test_macro_f1': manifest['metrics']['test']['macro_f1'],
    'test_recall': manifest['metrics']['test']['per_class_recall'],
}, indent=2))
"""
        ),
        code(
            """import shutil
from google.colab import files

archive = shutil.make_archive('/content/safestride_road_surface_model', 'zip', EXPORT_DIR)
files.download(archive)
"""
        ),
    ]
    return {
        "cells": cells,
        "metadata": {
            "accelerator": "GPU",
            "colab": {
                "name": "SafeStride road surface training",
                "provenance": [],
            },
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3",
            },
            "language_info": {"name": "python", "version": "3.x"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def main() -> None:
    NOTEBOOK_PATH.parent.mkdir(parents=True, exist_ok=True)
    NOTEBOOK_PATH.write_text(
        json.dumps(build_notebook(), indent=2) + "\n",
        encoding="utf-8",
    )
    print(NOTEBOOK_PATH)


if __name__ == "__main__":
    main()
