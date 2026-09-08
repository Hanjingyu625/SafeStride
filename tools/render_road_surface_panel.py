#!/usr/bin/env python3
"""Render print-ready road-surface model figures from the deployment manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    ROOT / "raspberry_pi" / "road_surface_inference" / "model_manifest.json"
)
DEFAULT_OUTPUT_DIR = ROOT / "docs" / "assets" / "road_surface_panel"

INK = "#17222B"
MUTED = "#5F6B73"
BLUE = "#2E6FBB"
TEAL = "#168C83"
AMBER = "#D89424"
RED = "#CF5B4E"
GREEN = "#3D8F63"
LIGHT = "#F3F6F7"
LINE = "#D8E0E3"
WHITE = "#FFFFFF"

CLASS_LABELS_KO = {
    "smooth_paved": "평탄 포장",
    "rough_paved": "거친 포장",
    "block_paved": "블록 포장",
    "gravel": "자갈",
    "mud_dirt": "진흙·흙",
    "unpaved_mixed": "혼합 비포장",
    "wet_paved": "젖은 포장",
    "wet_unpaved": "젖은 비포장",
    "snow_ice": "눈·빙판",
}

CLASS_LABELS_EN = {
    "smooth_paved": "Smooth paved",
    "rough_paved": "Rough paved",
    "block_paved": "Block paved",
    "gravel": "Gravel",
    "mud_dirt": "Mud / dirt",
    "unpaved_mixed": "Mixed unpaved",
    "wet_paved": "Wet paved",
    "wet_unpaved": "Wet unpaved",
    "snow_ice": "Snow / ice",
}

SOURCE_LABELS = {
    "rscd_train": "RSCD train",
    "streetsurfacevis": "StreetSurfaceVis",
    "gtos_mobile": "GTOS-Mobile",
    "cycling_small": "Cycling",
    "rscd_test_50k": "RSCD test",
    "rscd_vali_20k": "RSCD validation",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dpi", type=int, default=300)
    return parser.parse_args()


def configure_font() -> bool:
    candidates = (
        Path("C:/Windows/Fonts/malgun.ttf"),
        Path("C:/Windows/Fonts/malgunbd.ttf"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
    )
    for path in candidates:
        if path.is_file():
            font_manager.fontManager.addfont(path)
            mpl.rcParams["font.family"] = font_manager.FontProperties(
                fname=path
            ).get_name()
            mpl.rcParams["axes.unicode_minus"] = False
            return True
    mpl.rcParams["font.family"] = "DejaVu Sans"
    return False


def load_manifest(path: Path) -> dict:
    with path.open(encoding="utf-8") as source:
        manifest = json.load(source)
    required = ("classes", "dataset", "metrics", "artifact")
    missing = [key for key in required if key not in manifest]
    if missing:
        raise ValueError(f"manifest is missing required keys: {missing}")
    return manifest


def add_card(
    fig: plt.Figure,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    value: str,
    detail: str,
    color: str,
) -> None:
    card = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.008,rounding_size=0.012",
        linewidth=1.0,
        edgecolor=LINE,
        facecolor=WHITE,
        transform=fig.transFigure,
    )
    fig.patches.append(card)
    fig.add_artist(
        plt.Line2D(
            [x, x],
            [y + 0.018, y + height - 0.018],
            transform=fig.transFigure,
            color=color,
            linewidth=5,
            solid_capstyle="round",
        )
    )
    fig.text(x + 0.025, y + height - 0.038, title, fontsize=12, color=MUTED)
    fig.text(
        x + 0.025,
        y + 0.055,
        value,
        fontsize=25,
        fontweight="bold",
        color=INK,
    )
    fig.text(x + 0.025, y + 0.023, detail, fontsize=10.5, color=MUTED)


def save_figure(fig: plt.Figure, output_base: Path, dpi: int) -> None:
    output_base.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(
        output_base.with_suffix(".png"),
        dpi=dpi,
        bbox_inches="tight",
        facecolor=WHITE,
    )
    fig.savefig(
        output_base.with_suffix(".pdf"),
        bbox_inches="tight",
        facecolor=WHITE,
    )
    plt.close(fig)


def render_summary(manifest: dict, output_dir: Path, dpi: int) -> None:
    dataset = manifest["dataset"]
    metrics = manifest["metrics"]["test"]
    training = manifest.get("training_config", {})
    calibration = manifest.get("confidence_calibration", {})
    artifact = manifest["artifact"]
    classes = manifest["classes"]

    fig = plt.figure(figsize=(16, 9), facecolor=WHITE)
    fig.text(
        0.055,
        0.935,
        "노면 인식 모델 학습 및 배포 개요",
        fontsize=27,
        fontweight="bold",
        color=INK,
    )
    fig.text(
        0.055,
        0.895,
        "SafeStride · MobileNetV3-Small · Raspberry Pi 실시간 추론",
        fontsize=13,
        color=MUTED,
    )

    cards = (
        (
            "학습 데이터",
            f"{dataset['total']:,}장",
            f"{len(dataset['source_counts'])}개 공개 데이터 출처",
            BLUE,
        ),
        (
            "분류 모델",
            f"{len(classes)}개 노면",
            f"{manifest['model_name']} · {manifest['input']['width']}×{manifest['input']['height']}",
            TEAL,
        ),
        (
            "시험 성능",
            f"F1 {metrics['macro_f1'] * 100:.1f}%",
            f"정확도 {metrics['accuracy'] * 100:.1f}% · 독립 시험 {dataset['split']['test']:,}장",
            AMBER,
        ),
        (
            "배포 결과",
            f"{artifact['size_bytes'] / 1024**2:.2f} MB",
            f"{manifest['quantization'].upper()} TorchScript · 배포 승인",
            GREEN,
        ),
    )
    card_width = 0.207
    for index, card in enumerate(cards):
        add_card(
            fig,
            0.055 + index * 0.232,
            0.705,
            card_width,
            0.135,
            *card,
        )

    ax_flow = fig.add_axes([0.055, 0.47, 0.89, 0.17])
    ax_flow.set_axis_off()
    flow = (
        ("공개·현장 데이터", "6 sources\n25,684 images", BLUE),
        ("전처리·증강", "RGB 224×224\nclass-balanced sampling", TEAL),
        (
            "미세조정",
            f"AdamW · LR {training.get('finetune_lr', 0.0003):.0e}\nEMA {training.get('ema_decay', 0.999)}",
            AMBER,
        ),
        (
            "성능·신뢰도 검증",
            f"Macro F1 {metrics['macro_f1'] * 100:.1f}%\nECE {calibration.get('validation_ece_after', 0) * 100:.2f}%",
            RED,
        ),
        ("Pi 배포", "Float32 TorchScript\nROS surface topic", GREEN),
    )
    node_width = 0.16
    node_y = 0.17
    for index, (title, detail, color) in enumerate(flow):
        x = 0.005 + index * 0.207
        patch = FancyBboxPatch(
            (x, node_y),
            node_width,
            0.66,
            boxstyle="round,pad=0.012,rounding_size=0.025",
            linewidth=1.3,
            edgecolor=color,
            facecolor=WHITE,
            transform=ax_flow.transAxes,
        )
        ax_flow.add_patch(patch)
        ax_flow.text(
            x + node_width / 2,
            0.62,
            title,
            ha="center",
            va="center",
            fontsize=12.2,
            fontweight="bold",
            color=INK,
            transform=ax_flow.transAxes,
        )
        ax_flow.text(
            x + node_width / 2,
            0.36,
            detail,
            ha="center",
            va="center",
            fontsize=9.7,
            color=MUTED,
            linespacing=1.4,
            transform=ax_flow.transAxes,
        )
        if index < len(flow) - 1:
            ax_flow.annotate(
                "",
                xy=(x + 0.198, 0.5),
                xytext=(x + node_width + 0.008, 0.5),
                xycoords=ax_flow.transAxes,
                arrowprops=dict(arrowstyle="-|>", color="#98A5AC", lw=1.8),
            )

    source_counts = dataset["source_counts"]
    source_items = sorted(source_counts.items(), key=lambda item: item[1])
    ax_sources = fig.add_axes([0.07, 0.095, 0.48, 0.30])
    labels = [SOURCE_LABELS.get(name, name) for name, _ in source_items]
    values = [value for _, value in source_items]
    bars = ax_sources.barh(labels, values, color=BLUE, height=0.58)
    ax_sources.set_title(
        "데이터 출처별 이미지 수", loc="left", fontsize=14, fontweight="bold", pad=12
    )
    ax_sources.set_xlim(0, max(values) * 1.16)
    ax_sources.grid(axis="x", color=LINE, linewidth=0.8)
    ax_sources.set_axisbelow(True)
    ax_sources.spines[:].set_visible(False)
    ax_sources.tick_params(axis="both", colors=MUTED, labelsize=10)
    for bar, value in zip(bars, values):
        ax_sources.text(
            value + max(values) * 0.015,
            bar.get_y() + bar.get_height() / 2,
            f"{value:,}",
            va="center",
            fontsize=9.5,
            color=INK,
        )

    backbone_lr = training.get("finetune_lr", 0.0003) * training.get(
        "backbone_lr_multiplier", 0.1
    )
    settings = (
        ("Architecture", manifest["model_name"]),
        ("Input", f"RGB {manifest['input']['width']}×{manifest['input']['height']}"),
        ("Optimizer / batch", f"AdamW / {training.get('batch_size', 128)}"),
        (
            "Learning rate",
            f"{training.get('finetune_lr', 0.0003):.0e} / backbone {backbone_lr:.0e}",
        ),
        (
            "Training schedule",
            f"max {training.get('finetune_epochs', 30)} epochs / early stop {training.get('early_stop_patience', 7)}",
        ),
        (
            "Regularization",
            f"WD {training.get('weight_decay', 0.0001):.0e} / smoothing {training.get('label_smoothing', 0.05)}",
        ),
        ("Model averaging", f"EMA {training.get('ema_decay', 0.999)}"),
        (
            "Train / val / test",
            f"{dataset['split']['train']:,} / {dataset['split']['validation']:,} / {dataset['split']['test']:,}",
        ),
    )
    ax_config = fig.add_axes([0.605, 0.088, 0.34, 0.305])
    ax_config.set_axis_off()
    panel = FancyBboxPatch(
        (0, 0),
        1,
        0.96,
        boxstyle="round,pad=0.012,rounding_size=0.02",
        linewidth=1,
        edgecolor=LINE,
        facecolor=LIGHT,
        transform=ax_config.transAxes,
    )
    ax_config.add_patch(panel)
    ax_config.text(
        0.055,
        0.855,
        "핵심 학습 설정",
        fontsize=14,
        fontweight="bold",
        color=INK,
        transform=ax_config.transAxes,
    )
    for index, (name, value) in enumerate(settings):
        y = 0.73 - index * 0.086
        ax_config.text(
            0.06,
            y,
            name,
            fontsize=9.4,
            color=MUTED,
            transform=ax_config.transAxes,
        )
        ax_config.text(
            0.43,
            y,
            value,
            fontsize=9.5,
            fontweight="bold",
            color=INK,
            transform=ax_config.transAxes,
        )

    fig.text(
        0.055,
        0.035,
        "제어 배율: smooth/rough/mud/unpaved 1.00×, block/gravel 0.95×, "
        "wet 0.85×, snow/ice 0.75×. INT8 PTQ는 성능 저하로 제외했습니다.",
        fontsize=10.5,
        color=MUTED,
    )
    save_figure(fig, output_dir / "road_surface_training_summary", dpi)


def merged_group_metrics(matrix: np.ndarray, indices: tuple[int, ...]) -> tuple[float, float, float]:
    true_positive = matrix[np.ix_(indices, indices)].sum()
    actual = matrix[list(indices), :].sum()
    predicted = matrix[:, list(indices)].sum()
    recall = true_positive / actual
    precision = true_positive / predicted
    f1 = 2 * precision * recall / (precision + recall)
    return float(precision), float(recall), float(f1)


def aggregate_classes(
    matrix: np.ndarray, merged_indices: tuple[int, ...]
) -> np.ndarray:
    merged = set(merged_indices)
    mapping: dict[int, int] = {}
    next_index = 1
    for old_index in range(matrix.shape[0]):
        if old_index in merged:
            mapping[old_index] = 0
        else:
            mapping[old_index] = next_index
            next_index += 1

    aggregated = np.zeros((next_index, next_index), dtype=float)
    for actual in range(matrix.shape[0]):
        for predicted in range(matrix.shape[1]):
            aggregated[mapping[actual], mapping[predicted]] += matrix[
                actual, predicted
            ]
    return aggregated


def metrics_from_confusion(matrix: np.ndarray) -> tuple[float, float]:
    accuracy = float(np.trace(matrix) / matrix.sum())
    f1_scores: list[float] = []
    for index in range(matrix.shape[0]):
        true_positive = matrix[index, index]
        predicted = matrix[:, index].sum()
        actual = matrix[index, :].sum()
        precision = true_positive / predicted if predicted else 0.0
        recall = true_positive / actual if actual else 0.0
        denominator = precision + recall
        f1_scores.append(2 * precision * recall / denominator if denominator else 0.0)
    return accuracy, float(np.mean(f1_scores))


def render_performance(manifest: dict, output_dir: Path, dpi: int) -> None:
    metrics = manifest["metrics"]["test"]
    classes = manifest["classes"]
    matrix = np.asarray(metrics["confusion_matrix"], dtype=float)
    normalized = np.divide(
        matrix,
        matrix.sum(axis=1, keepdims=True),
        out=np.zeros_like(matrix),
        where=matrix.sum(axis=1, keepdims=True) != 0,
    )
    labels = [CLASS_LABELS_EN.get(label, label) for label in classes]
    recalls = np.asarray([metrics["per_class_recall"][name] for name in classes])
    paved_precision, paved_recall, paved_f1 = merged_group_metrics(matrix, (0, 1))
    grouped_accuracy, grouped_macro_f1 = metrics_from_confusion(
        aggregate_classes(matrix, (0, 1))
    )

    fig = plt.figure(figsize=(16, 9.5), facecolor=WHITE)
    fig.text(
        0.055,
        0.945,
        "Road-Surface Classification Performance",
        fontsize=27,
        fontweight="bold",
        color=INK,
    )
    fig.text(
        0.055,
        0.908,
        "Held-out test set · Row-normalized confusion matrix",
        fontsize=12.5,
        color=MUTED,
    )
    fig.text(
        0.66,
        0.938,
        f"9-class accuracy  {metrics['accuracy'] * 100:.1f}%",
        fontsize=15,
        fontweight="bold",
        color=BLUE,
    )
    fig.text(
        0.82,
        0.938,
        f"9-class Macro F1  {metrics['macro_f1'] * 100:.1f}%",
        fontsize=15,
        fontweight="bold",
        color=TEAL,
    )

    ax_matrix = fig.add_axes([0.07, 0.205, 0.53, 0.645])
    image = ax_matrix.imshow(normalized * 100, cmap="Blues", vmin=0, vmax=100)
    ax_matrix.set_xticks(range(len(labels)), labels, rotation=38, ha="right")
    ax_matrix.set_yticks(range(len(labels)), labels)
    ax_matrix.set_xlabel("Predicted", fontsize=12, labelpad=10)
    ax_matrix.set_ylabel("Actual", fontsize=12, labelpad=10)
    ax_matrix.tick_params(axis="both", labelsize=10, colors=INK)
    for row in range(normalized.shape[0]):
        for column in range(normalized.shape[1]):
            value = normalized[row, column] * 100
            if value < 0.5:
                continue
            ax_matrix.text(
                column,
                row,
                f"{value:.0f}",
                ha="center",
                va="center",
                fontsize=9.5,
                fontweight="bold" if row == column else "normal",
                color=WHITE if value >= 55 else INK,
            )
    colorbar = fig.colorbar(image, ax=ax_matrix, fraction=0.045, pad=0.035)
    colorbar.ax.tick_params(colors=MUTED)
    colorbar.outline.set_visible(False)

    ax_recall = fig.add_axes([0.68, 0.43, 0.27, 0.42])
    y = np.arange(len(labels))
    bar_colors = [TEAL if value >= 0.9 else AMBER if value >= 0.8 else RED for value in recalls]
    bars = ax_recall.barh(y, recalls * 100, color=bar_colors, height=0.6)
    ax_recall.set_yticks(y, labels)
    ax_recall.invert_yaxis()
    ax_recall.set_xlim(0, 105)
    ax_recall.set_title("Recall by class", loc="left", fontsize=14, fontweight="bold", pad=12)
    ax_recall.set_xlabel("Recall (%)", color=MUTED)
    ax_recall.grid(axis="x", color=LINE, linewidth=0.8)
    ax_recall.set_axisbelow(True)
    ax_recall.spines[:].set_visible(False)
    ax_recall.tick_params(axis="both", colors=MUTED, labelsize=9.5)
    for bar, value in zip(bars, recalls):
        ax_recall.text(
            min(value * 100 + 1.5, 100.5),
            bar.get_y() + bar.get_height() / 2,
            f"{value * 100:.1f}",
            va="center",
            fontsize=9.5,
            color=INK,
        )

    ax_note = fig.add_axes([0.66, 0.15, 0.30, 0.20])
    ax_note.set_axis_off()
    note = FancyBboxPatch(
        (0, 0),
        1,
        1,
        boxstyle="round,pad=0.015,rounding_size=0.025",
        facecolor=LIGHT,
        edgecolor=LINE,
        linewidth=1,
        transform=ax_note.transAxes,
    )
    ax_note.add_patch(note)
    ax_note.text(
        0.06,
        0.76,
        "Merged dry-paved control group",
        fontsize=13,
        fontweight="bold",
        color=INK,
        transform=ax_note.transAxes,
    )
    ax_note.text(
        0.06,
        0.48,
        f"8-group accuracy {grouped_accuracy * 100:.1f}%  ·  Macro F1 {grouped_macro_f1 * 100:.1f}%",
        fontsize=16,
        fontweight="bold",
        color=TEAL,
        transform=ax_note.transAxes,
    )
    smooth_to_rough = normalized[0, 1] * 100
    ax_note.text(
        0.06,
        0.18,
        f"Dry-paved recall {paved_recall * 100:.1f}%  ·  F1 {paved_f1 * 100:.1f}%\n"
        f"Main confusion: smooth → rough {smooth_to_rough:.1f}%. "
        "Both labels use the same base speed.",
        fontsize=10,
        color=MUTED,
        linespacing=1.5,
        transform=ax_note.transAxes,
    )

    fig.text(
        0.030,
        0.055,
        "Diagonal cells are correct predictions. Each row is normalized to 100% so classes with different support can be compared.",
        fontsize=10.5,
        color=MUTED,
    )
    save_figure(fig, output_dir / "road_surface_test_performance", dpi)


def main() -> None:
    args = parse_args()
    has_korean_font = configure_font()
    manifest = load_manifest(args.manifest.resolve())
    output_dir = args.output_dir.resolve()
    render_summary(manifest, output_dir, args.dpi)
    render_performance(manifest, output_dir, args.dpi)
    print(f"Rendered panel figures in {output_dir}")
    if not has_korean_font:
        print("Warning: no Korean font was found; install Malgun/Noto/Nanum font.")


if __name__ == "__main__":
    main()
