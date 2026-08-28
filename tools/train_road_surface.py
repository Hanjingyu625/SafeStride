#!/usr/bin/env python3
"""Train and export a ROS-compatible SafeStride surface classifier."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import csv
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import random
import re
import shutil
import statistics
import time
from typing import Callable, Iterable, Sequence
import zipfile

from road_surface_labels import (
    TARGET_CLASSES,
    map_cycling_label,
    map_rscd_label,
    map_rtk_quality_label,
    map_streetsurfacevis,
)


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
RSCD_REPO_ID = "rezzzq/RSCD-1million"
STREETSURFACEVIS_RECORD = "11449977"
RTK_DATASET_ID = "ffwgjdfn86"
CYCLING_RECORD = "17838875"
DATASET_CACHE_SCHEMA = 1
TRAINING_CHECKPOINT_SCHEMA = 1
os.environ.setdefault("HF_HUB_DISABLE_XET", "1")
os.environ.setdefault("HF_XET_NUM_CONCURRENT_RANGE_GETS", "2")
RSCD_TRAIN_LABELS = (
    "dry_asphalt_severe",
    "dry_asphalt_slight",
    "dry_asphalt_smooth",
    "dry_concrete_severe",
    "dry_concrete_slight",
    "dry_concrete_smooth",
    "dry_gravel",
    "dry_mud",
    "fresh_snow",
    "ice",
    "melted_snow",
    "water_asphalt_severe",
    "water_asphalt_slight",
    "water_asphalt_smooth",
    "water_concrete_severe",
    "water_concrete_slight",
    "water_concrete_smooth",
    "water_gravel",
    "water_mud",
    "wet_asphalt_severe",
    "wet_asphalt_slight",
    "wet_asphalt_smooth",
    "wet_concrete_severe",
    "wet_concrete_slight",
    "wet_concrete_smooth",
    "wet_gravel",
    "wet_mud",
)


@dataclass(frozen=True)
class Record:
    path: str
    label: str
    source: str
    raw_label: str
    group: str
    split_hint: str = ""
    sha256: str = ""


@dataclass(frozen=True)
class Metrics:
    loss: float
    accuracy: float
    macro_f1: float
    per_class_precision: dict[str, float]
    per_class_recall: dict[str, float]
    per_class_f1: dict[str, float]
    support: dict[str, int]
    confusion_matrix: list[list[int]]


@dataclass
class CandidateResult:
    model_name: str
    parameter_count: int
    state_dict: dict[str, object]
    validation: Metrics
    test: Metrics
    history: list[dict[str, object]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=Path("/content/road_surface_training"),
    )
    parser.add_argument(
        "--export-dir",
        type=Path,
        default=Path("/content/road_surface_exports_v2"),
    )
    parser.add_argument(
        "--checkpoint-dir",
        type=Path,
        help="Persist epoch checkpoints here so interrupted runs can resume.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume a compatible checkpoint from --checkpoint-dir.",
    )
    parser.add_argument(
        "--dataset-manifest",
        type=Path,
        help="Reuse a previously prepared dataset_manifest.csv.",
    )
    parser.add_argument(
        "--rebuild-dataset-cache",
        action="store_true",
        help="Ignore the prepared dataset cache and collect/validate again.",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--head-epochs", type=int, default=3)
    parser.add_argument(
        "--finetune-epochs",
        type=int,
        default=25,
        help="Maximum full-network epochs; early stopping selects the actual count.",
    )
    parser.add_argument("--early-stop-patience", type=int, default=6)
    parser.add_argument("--lr-plateau-patience", type=int, default=2)
    parser.add_argument("--lr-decay-factor", type=float, default=0.5)
    parser.add_argument("--min-lr", type=float, default=1.0e-6)
    parser.add_argument("--head-lr", type=float, default=1.0e-3)
    parser.add_argument("--finetune-lr", type=float, default=1.0e-4)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--label-smoothing", type=float, default=0.05)
    parser.add_argument(
        "--min-per-class",
        type=int,
        default=60,
        help="Hard floor needed to train and evaluate a class.",
    )
    parser.add_argument(
        "--recommended-per-class",
        type=int,
        default=250,
        help="Warn below this target, but do not abort training.",
    )
    parser.add_argument("--max-per-source-class", type=int, default=1600)
    parser.add_argument("--max-per-class", type=int, default=4000)
    parser.add_argument("--rscd-per-class", type=int, default=1200)
    parser.add_argument("--rscd-validation-per-class", type=int, default=100)
    parser.add_argument("--rscd-test-per-class", type=int, default=100)
    parser.add_argument("--rscd-download-workers", type=int, default=4)
    parser.add_argument(
        "--min-validation-per-class",
        type=int,
        default=10,
        help="Minimum validation and test support required for every class.",
    )
    parser.add_argument("--min-macro-f1", type=float, default=0.75)
    parser.add_argument("--min-class-recall", type=float, default=0.55)
    parser.add_argument(
        "--model-selection-tolerance",
        type=float,
        default=0.005,
        help="Prefer the smaller model only when validation macro F1 is this close.",
    )
    parser.add_argument(
        "--models",
        nargs="+",
        choices=(
            "mobilenet_v3_small",
            "mobilenet_v3_large",
            "efficientnet_b0",
        ),
        default=("mobilenet_v3_small",),
    )
    parser.add_argument("--skip-rscd", action="store_true")
    parser.add_argument("--skip-streetsurfacevis", action="store_true")
    parser.add_argument("--skip-rtk", action="store_true")
    parser.add_argument("--skip-cycling", action="store_true")
    parser.add_argument(
        "--local-data-dir",
        type=Path,
        help=(
            "Optional local data arranged as CLASS/GROUP/images. Group folders "
            "keep adjacent frames in the same split."
        ),
    )
    parser.add_argument("--quantize-int8", action="store_true")
    parser.add_argument("--calibration-batches", type=int, default=32)
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)
    try:
        import numpy as np
        import torch
    except ImportError:
        return
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True, warn_only=True)


def stable_seed(seed: int, value: object) -> int:
    digest = hashlib.sha256(str(value).encode("utf-8")).digest()
    return seed + int.from_bytes(digest[:4], "little")


def image_files(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def download_file(urls: str | Sequence[str], destination: Path) -> Path:
    import requests
    from tqdm.auto import tqdm

    if destination.is_file() and destination.stat().st_size > 0:
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    candidates = [urls] if isinstance(urls, str) else list(urls)
    temporary = destination.with_suffix(destination.suffix + ".part")
    last_error: Exception | None = None
    for url in candidates:
        try:
            temporary.unlink(missing_ok=True)
            with requests.get(
                url,
                stream=True,
                allow_redirects=True,
                timeout=90,
                headers={"User-Agent": "SafeStride training/2"},
            ) as response:
                response.raise_for_status()
                size = int(response.headers.get("content-length", 0))
                with temporary.open("wb") as output:
                    with tqdm(
                        total=size,
                        unit="B",
                        unit_scale=True,
                        desc=destination.name,
                    ) as progress:
                        for chunk in response.iter_content(1024 * 1024):
                            if chunk:
                                output.write(chunk)
                                progress.update(len(chunk))
            temporary.replace(destination)
            return destination
        except Exception as error:  # noqa: BLE001
            last_error = error
            print(f"download failed: {url}: {error!r}")
    raise RuntimeError(f"all downloads failed for {destination}") from last_error


def extract_zip_once(archive: Path, destination: Path) -> Path:
    marker = destination / ".extracted"
    if marker.is_file():
        return destination
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    root = destination.resolve()
    with zipfile.ZipFile(archive) as zipped:
        for member in zipped.infolist():
            target = (destination / member.filename).resolve()
            if root != target and root not in target.parents:
                raise RuntimeError(
                    f"unsafe path in {archive.name}: {member.filename}"
                )
        zipped.extractall(destination)
    marker.write_text("ok\n", encoding="ascii")
    return destination


def numeric_group(path: Path, block_size: int = 30) -> str:
    numbers = re.findall(r"\d+", path.stem)
    if not numbers:
        return f"{path.parent.name}:{path.stem}"
    number = int(numbers[-1][-9:])
    return f"{path.parent.name}:{number // block_size}"


def collect_rscd(
    work_dir: Path,
    train_per_class: int,
    validation_per_class: int,
    test_per_class: int,
    seed: int,
    download_workers: int,
) -> list[Record]:
    from huggingface_hub import HfApi, hf_hub_download
    from tqdm.auto import tqdm

    api = HfApi()
    revision = api.dataset_info(repo_id=RSCD_REPO_ID).sha
    if not revision:
        raise RuntimeError("Hugging Face did not return an RSCD revision")
    (work_dir / "rscd_revision.txt").write_text(
        revision + "\n", encoding="ascii"
    )
    print(f"RSCD revision: {revision}")
    records: list[Record] = []
    local_dir = work_dir / "rscd_hf"
    supported_targets = {
        label
        for raw in RSCD_TRAIN_LABELS
        if (label := map_rscd_label(raw)) is not None
    }

    def train_raw_limits(limit_per_class: int) -> dict[str, int]:
        raw_by_target: dict[str, list[str]] = {}
        for raw in RSCD_TRAIN_LABELS:
            label = map_rscd_label(raw)
            if label is not None:
                raw_by_target.setdefault(label, []).append(raw)

        limits: dict[str, int] = {}
        for raw_labels in raw_by_target.values():
            base, remainder = divmod(limit_per_class, len(raw_labels))
            for index, raw in enumerate(raw_labels):
                limits[raw] = base + (1 if index < remainder else 0)
        return limits

    def add_split(
        split_dir: str,
        limit_per_class: int,
        split_hint: str,
    ) -> None:
        print(f"indexing RSCD {split_dir}")
        by_class: dict[str, list[tuple[str, str]]] = {
            label: [] for label in TARGET_CLASSES
        }
        if split_dir == "train":
            raw_limits = train_raw_limits(limit_per_class)
            roots = [
                (f"train/{raw}", raw, raw_limits[raw])
                for raw in RSCD_TRAIN_LABELS
                if raw_limits[raw] > 0
            ]
        else:
            roots = [(split_dir, "", 0)]
        scanned = 0
        for root, train_raw, train_limit in roots:
            kept_from_root = 0
            entries = api.list_repo_tree(
                repo_id=RSCD_REPO_ID,
                repo_type="dataset",
                path_in_repo=root,
                recursive=True,
                revision=revision,
            )
            for entry in entries:
                scanned += 1
                relative = str(getattr(entry, "path", ""))
                if Path(relative).suffix.lower() not in IMAGE_EXTENSIONS:
                    continue
                if split_dir == "train":
                    raw = train_raw
                else:
                    stem = Path(relative).stem
                    raw = stem.split("-", 1)[1] if "-" in stem else stem
                label = map_rscd_label(raw)
                if label is not None and len(by_class[label]) < limit_per_class:
                    by_class[label].append((relative, raw))
                    kept_from_root += 1
                if split_dir == "train" and kept_from_root >= train_limit:
                    break
                if split_dir != "train" and all(
                    len(by_class[label]) >= limit_per_class
                    for label in supported_targets
                ):
                    break
                if split_dir != "train" and scanned % 5000 == 0:
                    counts = ", ".join(
                        f"{label}={len(by_class[label])}"
                        for label in sorted(supported_targets)
                    )
                    print(
                        f"RSCD {split_dir}: scanned {scanned} files ({counts})",
                        flush=True,
                    )
            if split_dir == "train":
                print(
                    f"RSCD {root}: kept {kept_from_root}/{train_limit}",
                    flush=True,
                )
            elif all(
                len(by_class[label]) >= limit_per_class
                for label in supported_targets
            ):
                break

        print(
            f"RSCD {split_dir}: finished indexing after {scanned} entries",
            flush=True,
        )

        selected_items: list[tuple[str, str, str]] = []
        for label in TARGET_CLASSES:
            items = by_class[label]
            random.Random(
                stable_seed(seed, ("rscd", split_dir, label))
            ).shuffle(items)
            selected = items[:limit_per_class]
            if not selected:
                print(
                    f"RSCD {split_dir} has no mapped samples for {label}",
                    flush=True,
                )
            selected_items.extend(
                (label, relative, raw) for relative, raw in selected
            )

        def download_item(item: tuple[str, str, str]) -> Record:
            label, relative, raw = item
            path = Path(
                hf_hub_download(
                    repo_id=RSCD_REPO_ID,
                    repo_type="dataset",
                    filename=relative,
                    local_dir=str(local_dir),
                    revision=revision,
                )
            )
            return Record(
                str(path),
                label,
                f"rscd_{split_dir}",
                raw,
                f"rscd:{split_dir}:{clean_group(raw)}:"
                f"{numeric_group(path, 60)}",
                split_hint,
            )

        print(
            f"downloading {len(selected_items)} RSCD {split_dir} images",
            flush=True,
        )
        with ThreadPoolExecutor(max_workers=download_workers) as executor:
            for record in tqdm(
                executor.map(download_item, selected_items),
                total=len(selected_items),
                desc=f"RSCD {split_dir}",
            ):
                records.append(record)

    add_split("train", train_per_class, "train")
    add_split("vali_20k", validation_per_class, "validation")
    add_split("test_50k", test_per_class, "test")
    return records


def clean_group(value: object) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", str(value)).strip("_")


def pick_column(columns: Iterable[object], names: Sequence[str]) -> str | None:
    normalized = {str(column).lower(): str(column) for column in columns}
    for name in names:
        if name in normalized:
            return normalized[name]
    for column in normalized.values():
        lowered = column.lower()
        if any(name in lowered for name in names):
            return column
    return None


def normalize_identifier(value: object) -> str:
    text = str(value).strip()
    if text.endswith(".0") and text[:-2].isdigit():
        text = text[:-2]
    return Path(text).stem


def dataset_split_hint(value: object) -> str:
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "1.0", "yes"}:
        return "train"
    if normalized in {"false", "0", "0.0", "no"}:
        return "test"
    return ""


def collect_streetsurfacevis(raw_dir: Path, work_dir: Path) -> list[Record]:
    import pandas as pd

    image_archive = download_file(
        (
            "https://zenodo.org/api/records/"
            f"{STREETSURFACEVIS_RECORD}/files/s_256.zip/content",
            "https://zenodo.org/records/"
            f"{STREETSURFACEVIS_RECORD}/files/s_256.zip?download=1",
        ),
        raw_dir / "streetsurfacevis_s256.zip",
    )
    csv_path = download_file(
        (
            "https://zenodo.org/api/records/"
            f"{STREETSURFACEVIS_RECORD}/files/"
            "streetSurfaceVis_v1_0.csv/content",
            "https://zenodo.org/records/"
            f"{STREETSURFACEVIS_RECORD}/files/"
            "streetSurfaceVis_v1_0.csv?download=1",
        ),
        raw_dir / "streetSurfaceVis_v1_0.csv",
    )
    root = extract_zip_once(image_archive, work_dir / "streetsurfacevis")
    frame = pd.read_csv(csv_path)
    surface_column = pick_column(frame.columns, ("surface_type",))
    quality_column = pick_column(
        frame.columns, ("surface_quality", "smoothness", "quality")
    )
    id_column = pick_column(
        frame.columns,
        ("mapillary_image_id", "image_id", "filename", "file_name", "id"),
    )
    latitude_column = pick_column(frame.columns, ("latitude", "lat"))
    longitude_column = pick_column(frame.columns, ("longitude", "lon", "lng"))
    user_column = pick_column(frame.columns, ("user_id", "user_name"))
    train_column = pick_column(frame.columns, ("train",))
    required = (surface_column, quality_column, id_column)
    if any(column is None for column in required):
        raise RuntimeError(
            "StreetSurfaceVis schema changed: " + ", ".join(frame.columns)
        )

    paths = {path.stem: path for path in image_files(root)}
    records: list[Record] = []
    for _, row in frame.iterrows():
        path = paths.get(normalize_identifier(row[id_column]))
        if path is None:
            continue
        label = map_streetsurfacevis(
            row[surface_column], row[quality_column]
        )
        if label is None:
            continue
        raw = f"{row[surface_column]}_{row[quality_column]}"
        if latitude_column and longitude_column:
            try:
                latitude = round(float(row[latitude_column]), 2)
                longitude = round(float(row[longitude_column]), 2)
                group = f"ssv:geo:{latitude:.2f}:{longitude:.2f}"
            except (TypeError, ValueError):
                group = f"ssv:user:{row.get(user_column, 'unknown')}"
        else:
            group = f"ssv:user:{row.get(user_column, 'unknown')}"
        records.append(
            Record(
                str(path),
                label,
                "streetsurfacevis",
                raw,
                group,
                dataset_split_hint(row[train_column])
                if train_column is not None
                else "",
            )
        )
    return records


def collect_rtk(raw_dir: Path, work_dir: Path) -> list[Record]:
    archive = download_file(
        "https://api.data.mendeley.com/datasets/"
        f"{RTK_DATASET_ID}/zip/file_downloaded?version=1",
        raw_dir / "rtk_quality.zip",
    )
    root = extract_zip_once(archive, work_dir / "rtk_quality")
    records: list[Record] = []
    for path in image_files(root):
        raw = path.parent.name
        label = map_rtk_quality_label(raw)
        if label is None:
            continue
        records.append(
            Record(
                str(path),
                label,
                "rtk_quality",
                raw,
                f"rtk:{numeric_group(path)}",
            )
        )
    return records


def collect_cycling(raw_dir: Path, work_dir: Path) -> list[Record]:
    archive = download_file(
        f"https://zenodo.org/records/{CYCLING_RECORD}/files/"
        "cycling_street_surface_dataset.zip?download=1",
        raw_dir / "cycling_street_surface_dataset.zip",
    )
    root = extract_zip_once(archive, work_dir / "cycling")
    candidates: list[Record] = []
    for path in image_files(root):
        raw = path.parent.name
        label = map_cycling_label(raw)
        if label is None:
            continue
        candidates.append(
            Record(
                str(path),
                label,
                "cycling_small",
                raw,
                f"cycling:{numeric_group(path)}",
            )
        )

    # The archive contains multiple resolutions of the same captured frame.
    # Keep one canonical copy so a frame cannot leak across data splits.
    canonical: dict[tuple[str, str], Record] = {}
    for record in candidates:
        path = Path(record.path)
        key = (record.label, path.stem)
        current = canonical.get(key)
        if current is None or path.stat().st_size > Path(current.path).stat().st_size:
            canonical[key] = record
    return list(canonical.values())


def collect_local(root: Path) -> list[Record]:
    if not root.is_dir():
        raise RuntimeError(f"local data directory does not exist: {root}")
    records: list[Record] = []
    for label in TARGET_CLASSES:
        class_dir = root / label
        if not class_dir.is_dir():
            continue
        for path in image_files(class_dir):
            relative = path.relative_to(class_dir)
            group_name = relative.parts[0] if len(relative.parts) > 1 else path.stem
            records.append(
                Record(
                    str(path),
                    label,
                    "safestride_local",
                    label,
                    f"local:{label}:{clean_group(group_name)}",
                )
            )
    return records


def cap_records(
    records: Sequence[Record],
    max_per_source_class: int,
    max_per_class: int,
    seed: int,
) -> list[Record]:
    by_source: dict[tuple[str, str, str], list[Record]] = {}
    for record in records:
        by_source.setdefault(
            (record.source, record.label, record.split_hint), []
        ).append(record)
    capped: list[Record] = []
    for key, items in sorted(by_source.items()):
        random.Random(stable_seed(seed, key)).shuffle(items)
        capped.extend(items[:max_per_source_class])

    by_class: dict[str, list[Record]] = {}
    for record in capped:
        by_class.setdefault(record.label, []).append(record)
    final: list[Record] = []
    for label in TARGET_CLASSES:
        items = by_class.get(label, [])
        protected = [
            record
            for record in items
            if record.split_hint in {"validation", "test"}
        ]
        candidates = [
            record
            for record in items
            if record.split_hint not in {"validation", "test"}
        ]
        random.Random(stable_seed(seed, ("class", label))).shuffle(candidates)
        remaining = max(max_per_class - len(protected), 0)
        final.extend(protected[:max_per_class])
        final.extend(candidates[:remaining])
    random.Random(seed).shuffle(final)
    return final


def validate_and_deduplicate(records: Sequence[Record]) -> list[Record]:
    from PIL import Image
    from tqdm.auto import tqdm

    valid: list[Record] = []
    hashes: dict[str, Record] = {}
    for record in tqdm(records, desc="validating images"):
        path = Path(record.path)
        try:
            with Image.open(path) as image:
                image.verify()
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
        except Exception as error:  # noqa: BLE001
            print(f"invalid image skipped: {path}: {error}")
            continue
        previous = hashes.get(digest)
        if previous is not None:
            if previous.label != record.label:
                raise RuntimeError(
                    "duplicate image has conflicting labels: "
                    f"{previous.path}={previous.label}, {record.path}={record.label}"
                )
            continue
        hashes[digest] = record
        valid.append(replace(record, sha256=digest))
    return valid


def count_by_class(records: Sequence[Record]) -> dict[str, int]:
    counts = {label: 0 for label in TARGET_CLASSES}
    for record in records:
        counts[record.label] += 1
    return counts


def count_by_source(records: Sequence[Record]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for record in records:
        counts[record.source] = counts.get(record.source, 0) + 1
    return dict(sorted(counts.items()))


def require_class_coverage(
    records: Sequence[Record], minimum: int, recommended: int
) -> None:
    counts = count_by_class(records)
    missing = {label: count for label, count in counts.items() if count < minimum}
    below_target = {
        label: count for label, count in counts.items() if count < recommended
    }
    print("class counts:", json.dumps(counts, indent=2))
    if missing:
        raise RuntimeError(
            "training cannot evaluate classes below the hard data floor: "
            + ", ".join(f"{label}={count}" for label, count in missing.items())
            + f"; each class requires at least {minimum} valid images"
        )
    if below_target:
        print(
            "data coverage warning (training will continue): "
            + ", ".join(
                f"{label}={count}" for label, count in below_target.items()
            )
            + f"; recommended target is {recommended} images per class"
        )


def require_split_coverage(
    records: Sequence[Record],
    splits: Sequence[tuple[str, Sequence[int], int]],
) -> None:
    failures: list[str] = []
    for name, indices, minimum in splits:
        subset = [records[index] for index in indices]
        counts = count_by_class(subset)
        print(f"{name}: {len(indices)} {counts}")
        failures.extend(
            f"{name}/{label}={count} < {minimum}"
            for label, count in counts.items()
            if count < minimum
        )
    if failures:
        raise RuntimeError(
            "grouped split does not contain enough independent examples: "
            + ", ".join(failures)
        )


def stratified_group_split(
    records: Sequence[Record], seed: int
) -> tuple[list[int], list[int], list[int]]:
    import numpy as np
    from sklearn.model_selection import StratifiedGroupKFold

    forced_train = [
        index for index, record in enumerate(records) if record.split_hint == "train"
    ]
    forced_validation = [
        index
        for index, record in enumerate(records)
        if record.split_hint == "validation"
    ]
    forced_test = [
        index for index, record in enumerate(records) if record.split_hint == "test"
    ]
    available = [
        index for index, record in enumerate(records) if not record.split_hint
    ]
    if not available:
        return forced_train, forced_validation, forced_test

    labels = np.asarray([records[index].label for index in available])
    groups = np.asarray([records[index].group for index in available])
    indices = np.arange(len(available))
    splitter = StratifiedGroupKFold(n_splits=5, shuffle=True, random_state=seed)
    folds = list(splitter.split(indices, labels, groups))
    _, test_positions = folds[0]
    _, validation_positions = folds[1]
    held_out = set(test_positions.tolist()) | set(validation_positions.tolist())
    train_positions = [index for index in indices.tolist() if index not in held_out]
    train_indices = forced_train + [available[index] for index in train_positions]
    validation_indices = forced_validation + [
        available[index] for index in validation_positions.tolist()
    ]
    test_indices = forced_test + [
        available[index] for index in test_positions.tolist()
    ]
    return train_indices, validation_indices, test_indices


def write_records(
    path: Path,
    records: Sequence[Record],
    split_by_index: dict[int, str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=(
                "path",
                "label",
                "source",
                "raw_label",
                "group",
                "split_hint",
                "sha256",
                "split",
            ),
        )
        writer.writeheader()
        for index, record in enumerate(records):
            row = asdict(record)
            row["split"] = split_by_index[index]
            writer.writerow(row)


def read_records(
    path: Path,
) -> tuple[list[Record], tuple[list[int], list[int], list[int]]]:
    records: list[Record] = []
    split_indices: dict[str, list[int]] = {
        "train": [],
        "validation": [],
        "test": [],
    }
    with path.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            split = str(row.get("split", ""))
            if split not in split_indices:
                raise RuntimeError(f"invalid split {split!r} in {path}")
            record = Record(
                path=str(row["path"]),
                label=str(row["label"]),
                source=str(row["source"]),
                raw_label=str(row["raw_label"]),
                group=str(row["group"]),
                split_hint=str(row.get("split_hint", "")),
                sha256=str(row.get("sha256", "")),
            )
            if record.label not in TARGET_CLASSES:
                raise RuntimeError(
                    f"unknown class {record.label!r} in {path}"
                )
            split_indices[split].append(len(records))
            records.append(record)
    if not records or any(not split_indices[name] for name in split_indices):
        raise RuntimeError(f"incomplete dataset manifest: {path}")
    return records, (
        split_indices["train"],
        split_indices["validation"],
        split_indices["test"],
    )


def dataset_cache_config(args: argparse.Namespace) -> dict[str, object]:
    return {
        "schema": DATASET_CACHE_SCHEMA,
        "seed": args.seed,
        "classes": list(TARGET_CLASSES),
        "max_per_source_class": args.max_per_source_class,
        "max_per_class": args.max_per_class,
        "rscd_per_class": args.rscd_per_class,
        "rscd_validation_per_class": args.rscd_validation_per_class,
        "rscd_test_per_class": args.rscd_test_per_class,
        "skip_rscd": args.skip_rscd,
        "skip_streetsurfacevis": args.skip_streetsurfacevis,
        "skip_rtk": args.skip_rtk,
        "skip_cycling": args.skip_cycling,
        "local_data_dir": (
            str(args.local_data_dir.resolve())
            if args.local_data_dir is not None
            else None
        ),
    }


def prepared_dataset_paths(work_dir: Path) -> tuple[Path, Path]:
    cache_dir = work_dir / "prepared"
    return cache_dir / "dataset_manifest.csv", cache_dir / "dataset_cache.json"


def load_prepared_dataset(
    work_dir: Path, args: argparse.Namespace
) -> tuple[list[Record], tuple[list[int], list[int], list[int]]] | None:
    manifest_path, metadata_path = prepared_dataset_paths(work_dir)
    if args.rebuild_dataset_cache or not (
        manifest_path.is_file() and metadata_path.is_file()
    ):
        return None
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("config") != dataset_cache_config(args):
            print("prepared dataset cache configuration changed; rebuilding")
            return None
        records, splits = read_records(manifest_path)
    except Exception as error:  # noqa: BLE001
        print(f"prepared dataset cache is unusable; rebuilding: {error!r}")
        return None
    print(
        f"reusing prepared dataset cache: {manifest_path} "
        f"({len(records)} images)",
        flush=True,
    )
    return records, splits


def save_prepared_dataset(
    work_dir: Path,
    args: argparse.Namespace,
    records: Sequence[Record],
    splits: tuple[list[int], list[int], list[int]],
) -> None:
    manifest_path, metadata_path = prepared_dataset_paths(work_dir)
    split_by_index = {
        **{index: "train" for index in splits[0]},
        **{index: "validation" for index in splits[1]},
        **{index: "test" for index in splits[2]},
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_manifest = manifest_path.with_suffix(".csv.tmp")
    temporary_metadata = metadata_path.with_suffix(".json.tmp")
    write_records(temporary_manifest, records, split_by_index)
    temporary_metadata.write_text(
        json.dumps(
            {
                "schema": DATASET_CACHE_SCHEMA,
                "config": dataset_cache_config(args),
                "record_count": len(records),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary_manifest.replace(manifest_path)
    temporary_metadata.replace(metadata_path)
    print(f"saved prepared dataset cache: {manifest_path}", flush=True)


def dataset_fingerprint(
    records: Sequence[Record],
    splits: tuple[list[int], list[int], list[int]],
) -> str:
    split_names = ("train", "validation", "test")
    split_by_index = {
        index: split_names[split_number]
        for split_number, indices in enumerate(splits)
        for index in indices
    }
    digest = hashlib.sha256()
    for index, record in enumerate(records):
        digest.update(
            "\0".join(
                (
                    record.path,
                    record.label,
                    record.source,
                    record.group,
                    record.sha256,
                    split_by_index[index],
                )
            ).encode("utf-8")
        )
        digest.update(b"\n")
    return digest.hexdigest()


def build_transforms(image_size: int):
    from torchvision import transforms

    train_transform = transforms.Compose(
        [
            transforms.Resize((256, 256)),
            transforms.RandomResizedCrop(
                image_size, scale=(0.72, 1.0), ratio=(0.85, 1.45)
            ),
            transforms.RandomHorizontalFlip(0.5),
            transforms.ColorJitter(
                brightness=0.20,
                contrast=0.20,
                saturation=0.10,
                hue=0.02,
            ),
            transforms.RandomApply(
                [transforms.GaussianBlur(kernel_size=3, sigma=(0.1, 1.2))],
                p=0.15,
            ),
            transforms.RandomPerspective(distortion_scale=0.08, p=0.15),
            transforms.ToTensor(),
            transforms.Normalize(IMAGENET_MEAN, IMAGENET_STD),
        ]
    )
    evaluation_transform = transforms.Compose(
        [
            transforms.Resize((image_size, image_size)),
            transforms.ToTensor(),
            transforms.Normalize(IMAGENET_MEAN, IMAGENET_STD),
        ]
    )
    return train_transform, evaluation_transform


class RoadSurfaceDataset:
    """Small adapter that delays heavy imports until training starts."""

    def __new__(
        cls,
        records: Sequence[Record],
        indices: Sequence[int],
        transform,
        label_to_index: dict[str, int],
    ):
        from PIL import Image
        from torch.utils.data import Dataset

        selected = [records[index] for index in indices]

        class DatasetImplementation(Dataset):
            def __len__(self) -> int:
                return len(selected)

            def __getitem__(self, index: int):
                record = selected[index]
                with Image.open(record.path) as image:
                    rgb = image.convert("RGB")
                    tensor = transform(rgb)
                return tensor, label_to_index[record.label]

        return DatasetImplementation()


def build_loaders(
    records: Sequence[Record],
    splits: tuple[list[int], list[int], list[int]],
    image_size: int,
    batch_size: int,
    num_workers: int,
    device_type: str,
    seed: int,
):
    import numpy as np
    import torch
    from torch.utils.data import DataLoader, WeightedRandomSampler

    label_to_index = {label: index for index, label in enumerate(TARGET_CLASSES)}
    train_transform, evaluation_transform = build_transforms(image_size)
    train_indices, validation_indices, test_indices = splits
    train_dataset = RoadSurfaceDataset(
        records, train_indices, train_transform, label_to_index
    )
    validation_dataset = RoadSurfaceDataset(
        records, validation_indices, evaluation_transform, label_to_index
    )
    test_dataset = RoadSurfaceDataset(
        records, test_indices, evaluation_transform, label_to_index
    )

    train_labels = [label_to_index[records[index].label] for index in train_indices]
    counts = np.bincount(train_labels, minlength=len(TARGET_CLASSES))
    weights = [1.0 / math.sqrt(max(int(counts[label]), 1)) for label in train_labels]
    generator = torch.Generator().manual_seed(seed)
    sampler = WeightedRandomSampler(
        weights,
        len(weights),
        replacement=True,
        generator=generator,
    )
    options = {
        "batch_size": batch_size,
        "num_workers": num_workers,
        "pin_memory": device_type == "cuda",
        "persistent_workers": num_workers > 0,
        "generator": generator,
    }
    train_loader = DataLoader(train_dataset, sampler=sampler, **options)
    validation_loader = DataLoader(validation_dataset, shuffle=False, **options)
    test_loader = DataLoader(test_dataset, shuffle=False, **options)
    return train_loader, validation_loader, test_loader


def select_calibration_indices(
    records: Sequence[Record],
    train_indices: Sequence[int],
    max_samples: int,
    seed: int,
) -> list[int]:
    """Select a deterministic, class-balanced subset from the train split."""

    if max_samples < 1:
        raise ValueError("calibration sample count must be positive")
    by_class: dict[str, list[int]] = {label: [] for label in TARGET_CLASSES}
    for index in train_indices:
        by_class[records[index].label].append(index)
    for label, indices in by_class.items():
        random.Random(stable_seed(seed, ("calibration", label))).shuffle(indices)

    selected: list[int] = []
    while len(selected) < max_samples:
        added = False
        for label in TARGET_CLASSES:
            indices = by_class[label]
            if indices:
                selected.append(indices.pop())
                added = True
                if len(selected) >= max_samples:
                    break
        if not added:
            break
    random.Random(stable_seed(seed, "calibration_order")).shuffle(selected)
    return selected


def build_calibration_loader(
    records: Sequence[Record],
    train_indices: Sequence[int],
    image_size: int,
    batch_size: int,
    num_workers: int,
    batches: int,
    seed: int,
):
    """Build a no-augmentation loader used only for static PTQ calibration."""

    from torch.utils.data import DataLoader

    label_to_index = {label: index for index, label in enumerate(TARGET_CLASSES)}
    _, evaluation_transform = build_transforms(image_size)
    selected = select_calibration_indices(
        records,
        train_indices,
        batches * batch_size,
        seed,
    )
    dataset = RoadSurfaceDataset(
        records,
        selected,
        evaluation_transform,
        label_to_index,
    )
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=False,
        persistent_workers=num_workers > 0,
    )
    class_counts = {
        label: sum(records[index].label == label for index in selected)
        for label in TARGET_CLASSES
    }
    return loader, {
        "split": "train",
        "transform": "resize_normalize_without_augmentation",
        "selected_samples": len(selected),
        "class_counts": class_counts,
    }


def build_model(name: str, class_count: int, pretrained: bool = True):
    import torch.nn as nn
    from torchvision.models import (
        EfficientNet_B0_Weights,
        MobileNet_V3_Large_Weights,
        MobileNet_V3_Small_Weights,
        efficientnet_b0,
        mobilenet_v3_large,
        mobilenet_v3_small,
    )

    if name == "mobilenet_v3_small":
        weights = MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
        model = mobilenet_v3_small(weights=weights)
    elif name == "mobilenet_v3_large":
        weights = MobileNet_V3_Large_Weights.DEFAULT if pretrained else None
        model = mobilenet_v3_large(weights=weights)
    elif name == "efficientnet_b0":
        weights = EfficientNet_B0_Weights.DEFAULT if pretrained else None
        model = efficientnet_b0(weights=weights)
    else:
        raise ValueError(f"unsupported model: {name}")
    model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, class_count)
    return model


def set_head_only(model, head_only: bool) -> None:
    for name, parameter in model.named_parameters():
        parameter.requires_grad = not head_only or name.startswith("classifier")


def metrics_from_predictions(
    loss: float,
    truth: Sequence[int],
    predictions: Sequence[int],
) -> Metrics:
    from sklearn.metrics import (
        accuracy_score,
        confusion_matrix,
        f1_score,
        precision_recall_fscore_support,
    )

    labels = list(range(len(TARGET_CLASSES)))
    precision, recall, class_f1, class_support = (
        precision_recall_fscore_support(
            truth,
            predictions,
            labels=labels,
            average=None,
            zero_division=0,
        )
    )
    return Metrics(
        loss=float(loss),
        accuracy=float(accuracy_score(truth, predictions)),
        macro_f1=float(
            f1_score(
                truth,
                predictions,
                labels=labels,
                average="macro",
                zero_division=0,
            )
        ),
        per_class_precision={
            label: float(precision[index])
            for index, label in enumerate(TARGET_CLASSES)
        },
        per_class_recall={
            label: float(recall[index])
            for index, label in enumerate(TARGET_CLASSES)
        },
        per_class_f1={
            label: float(class_f1[index])
            for index, label in enumerate(TARGET_CLASSES)
        },
        support={
            label: int(class_support[index])
            for index, label in enumerate(TARGET_CLASSES)
        },
        confusion_matrix=confusion_matrix(
            truth, predictions, labels=labels
        ).astype(int).tolist(),
    )


def run_epoch(model, loader, criterion, device, optimizer=None, scaler=None) -> Metrics:
    import torch

    training = optimizer is not None
    model.train(training)
    total_loss = 0.0
    total = 0
    truth: list[int] = []
    predictions: list[int] = []
    for images, labels in loader:
        images = images.to(device, non_blocking=True)
        labels = labels.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=device.type == "cuda",
            ):
                logits = model(images)
                loss = criterion(logits, labels)
            if training:
                assert scaler is not None
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()
        total_loss += float(loss.item()) * images.size(0)
        total += images.size(0)
        truth.extend(labels.detach().cpu().tolist())
        predictions.extend(logits.argmax(1).detach().cpu().tolist())
    if total == 0:
        raise RuntimeError("empty data loader")
    return metrics_from_predictions(total_loss / total, truth, predictions)


def training_checkpoint_signature(
    model_name: str,
    args: argparse.Namespace,
    prepared_dataset_fingerprint: str,
) -> dict[str, object]:
    return {
        "schema": TRAINING_CHECKPOINT_SCHEMA,
        "model_name": model_name,
        "classes": list(TARGET_CLASSES),
        "dataset_fingerprint": prepared_dataset_fingerprint,
        "seed": args.seed,
        "image_size": args.image_size,
        "batch_size": args.batch_size,
        "head_epochs": args.head_epochs,
        "finetune_epochs": args.finetune_epochs,
        "early_stop_patience": args.early_stop_patience,
        "lr_plateau_patience": args.lr_plateau_patience,
        "lr_decay_factor": args.lr_decay_factor,
        "min_lr": args.min_lr,
        "head_lr": args.head_lr,
        "finetune_lr": args.finetune_lr,
        "weight_decay": args.weight_decay,
        "label_smoothing": args.label_smoothing,
    }


def cpu_state_dict(model) -> dict[str, object]:
    return {
        name: tensor.detach().cpu().clone()
        for name, tensor in model.state_dict().items()
    }


def save_training_checkpoint(path: Path, payload: dict[str, object]) -> None:
    import torch

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(payload, temporary)
    temporary.replace(path)
    print(
        f"saved checkpoint: {path} "
        f"({payload['phase']} epoch {payload['epoch']})",
        flush=True,
    )


def load_training_checkpoint(
    path: Path,
    expected_signature: dict[str, object],
) -> dict[str, object] | None:
    import torch

    if not path.is_file():
        return None
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if checkpoint.get("signature") != expected_signature:
        raise RuntimeError(
            f"checkpoint configuration does not match this run: {path}; "
            "use a new checkpoint directory or remove the old checkpoint"
        )
    return checkpoint


def sampler_generator(loader):
    return getattr(getattr(loader, "sampler", None), "generator", None)


def train_candidate(
    model_name: str,
    loaders,
    args: argparse.Namespace,
    device,
    prepared_dataset_fingerprint: str,
) -> CandidateResult:
    import torch
    import torch.nn as nn

    train_loader, validation_loader, test_loader = loaders
    signature = training_checkpoint_signature(
        model_name, args, prepared_dataset_fingerprint
    )
    checkpoint_path = (
        args.checkpoint_dir / f"{model_name}.checkpoint.pt"
        if args.checkpoint_dir is not None
        else None
    )
    checkpoint = (
        load_training_checkpoint(checkpoint_path, signature)
        if args.resume and checkpoint_path is not None
        else None
    )
    model = build_model(
        model_name,
        len(TARGET_CLASSES),
        pretrained=checkpoint is None,
    )
    if checkpoint is not None:
        model.load_state_dict(checkpoint["model_state"])
    model.to(device)
    parameter_count = sum(parameter.numel() for parameter in model.parameters())
    criterion = nn.CrossEntropyLoss(label_smoothing=args.label_smoothing)
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")
    history: list[dict[str, object]] = (
        list(checkpoint["history"]) if checkpoint is not None else []
    )
    best_score = (
        float(checkpoint["best_score"]) if checkpoint is not None else -math.inf
    )
    best_loss = (
        float(checkpoint["best_loss"]) if checkpoint is not None else math.inf
    )
    best_state: dict[str, object] | None = (
        checkpoint.get("best_state") if checkpoint is not None else None
    )
    epochs_without_improvement = (
        int(checkpoint["epochs_without_improvement"])
        if checkpoint is not None
        else 0
    )

    phases = (
        ("head", args.head_epochs, args.head_lr, True),
        ("finetune", args.finetune_epochs, args.finetune_lr, False),
    )
    phase_names = [phase[0] for phase in phases]
    resume_phase = str(checkpoint["phase"]) if checkpoint is not None else None
    resume_phase_index = (
        len(phases)
        if checkpoint is not None and bool(checkpoint.get("complete"))
        else phase_names.index(resume_phase)
        if resume_phase in phase_names
        else 0
    )
    if checkpoint is not None:
        print(
            f"resuming {model_name} from {checkpoint_path}: "
            f"{resume_phase} epoch {checkpoint['epoch']}",
            flush=True,
        )

    for phase_index, (phase, epochs, learning_rate, head_only) in enumerate(phases):
        if phase_index < resume_phase_index:
            continue
        set_head_only(model, head_only)
        optimizer = torch.optim.AdamW(
            [parameter for parameter in model.parameters() if parameter.requires_grad],
            lr=learning_rate,
            weight_decay=args.weight_decay,
        )
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            optimizer,
            mode="max",
            factor=args.lr_decay_factor,
            patience=args.lr_plateau_patience,
            min_lr=args.min_lr,
        )
        start_epoch = 1
        resuming_this_phase = (
            checkpoint is not None
            and not bool(checkpoint.get("complete"))
            and phase == resume_phase
        )
        if resuming_this_phase:
            optimizer.load_state_dict(checkpoint["optimizer_state"])
            scheduler.load_state_dict(checkpoint["scheduler_state"])
            scaler.load_state_dict(checkpoint["scaler_state"])
            start_epoch = int(checkpoint["epoch"]) + 1
            generator = sampler_generator(train_loader)
            if generator is not None and checkpoint.get("sampler_state") is not None:
                generator.set_state(checkpoint["sampler_state"])
            if checkpoint.get("torch_rng_state") is not None:
                torch.set_rng_state(checkpoint["torch_rng_state"])
            if device.type == "cuda" and checkpoint.get("cuda_rng_state"):
                torch.cuda.set_rng_state_all(checkpoint["cuda_rng_state"])
            if checkpoint.get("python_rng_state") is not None:
                random.setstate(checkpoint["python_rng_state"])
        elif phase == "finetune":
            epochs_without_improvement = 0
        if phase == "finetune" and (
            epochs_without_improvement >= args.early_stop_patience
        ):
            print(f"{model_name}: early stopping was already reached")
            continue
        for epoch in range(start_epoch, epochs + 1):
            train_metrics = run_epoch(
                model, train_loader, criterion, device, optimizer, scaler
            )
            validation_metrics = run_epoch(
                model, validation_loader, criterion, device
            )
            scheduler.step(validation_metrics.macro_f1)
            current_lr = float(optimizer.param_groups[0]["lr"])
            row = {
                "model": model_name,
                "phase": phase,
                "epoch": epoch,
                "learning_rate": current_lr,
                "train": asdict(train_metrics),
                "validation": asdict(validation_metrics),
            }
            history.append(row)
            print(
                f"{model_name} {phase} {epoch:02d}: "
                f"train_f1={train_metrics.macro_f1:.3f} "
                f"val_f1={validation_metrics.macro_f1:.3f} "
                f"val_acc={validation_metrics.accuracy:.3f} "
                f"lr={current_lr:.2e}"
            )
            improved = (
                validation_metrics.macro_f1 > best_score + 1.0e-6
                or (
                    abs(validation_metrics.macro_f1 - best_score) <= 1.0e-6
                    and validation_metrics.loss < best_loss
                )
            )
            if improved:
                best_score = validation_metrics.macro_f1
                best_loss = validation_metrics.loss
                best_state = cpu_state_dict(model)
                epochs_without_improvement = 0
            elif phase == "finetune":
                epochs_without_improvement += 1
            if checkpoint_path is not None:
                generator = sampler_generator(train_loader)
                save_training_checkpoint(
                    checkpoint_path,
                    {
                        "schema": TRAINING_CHECKPOINT_SCHEMA,
                        "signature": signature,
                        "complete": False,
                        "phase": phase,
                        "epoch": epoch,
                        "model_state": cpu_state_dict(model),
                        "optimizer_state": optimizer.state_dict(),
                        "scheduler_state": scheduler.state_dict(),
                        "scaler_state": scaler.state_dict(),
                        "best_state": best_state,
                        "best_score": best_score,
                        "best_loss": best_loss,
                        "epochs_without_improvement": epochs_without_improvement,
                        "history": history,
                        "sampler_state": (
                            generator.get_state() if generator is not None else None
                        ),
                        "torch_rng_state": torch.get_rng_state(),
                        "cuda_rng_state": (
                            torch.cuda.get_rng_state_all()
                            if device.type == "cuda"
                            else None
                        ),
                        "python_rng_state": random.getstate(),
                    },
                )
            if (
                phase == "finetune"
                and epochs_without_improvement >= args.early_stop_patience
            ):
                print(f"{model_name}: early stopping")
                break

    if best_state is None:
        raise RuntimeError(f"{model_name} did not produce a checkpoint")
    model.load_state_dict(best_state)
    model.to(device).eval()
    if checkpoint_path is not None:
        save_training_checkpoint(
            checkpoint_path,
            {
                "schema": TRAINING_CHECKPOINT_SCHEMA,
                "signature": signature,
                "complete": True,
                "phase": "complete",
                "epoch": len(history),
                "model_state": best_state,
                "best_state": best_state,
                "best_score": best_score,
                "best_loss": best_loss,
                "epochs_without_improvement": epochs_without_improvement,
                "history": history,
            },
        )
    validation_metrics = run_epoch(model, validation_loader, criterion, device)
    test_metrics = run_epoch(model, test_loader, criterion, device)
    return CandidateResult(
        model_name=model_name,
        parameter_count=parameter_count,
        state_dict=best_state,
        validation=validation_metrics,
        test=test_metrics,
        history=history,
    )


def choose_candidate(
    results: Sequence[CandidateResult], tolerance: float
) -> CandidateResult:
    best_score = max(result.validation.macro_f1 for result in results)
    close = [
        result
        for result in results
        if best_score - result.validation.macro_f1 <= tolerance
    ]
    selected = min(close, key=lambda result: result.parameter_count)
    print(
        f"selected {selected.model_name}: "
        f"val_f1={selected.validation.macro_f1:.3f}, "
        f"test_f1={selected.test.macro_f1:.3f}, "
        f"params={selected.parameter_count / 1e6:.2f}M"
    )
    return selected


def export_torchscript(model, path: Path, image_size: int) -> None:
    import torch

    model = copy.deepcopy(model).cpu().eval()
    example = torch.randn(1, 3, image_size, image_size)
    with torch.inference_mode():
        expected = model(example)
        scripted = torch.jit.trace(model, example, strict=True)
        scripted = torch.jit.freeze(scripted.eval())
        actual = scripted(example)
    if tuple(actual.shape) != (1, len(TARGET_CLASSES)):
        raise RuntimeError(f"invalid TorchScript output shape: {tuple(actual.shape)}")
    if not torch.allclose(expected, actual, rtol=1.0e-4, atol=1.0e-5):
        raise RuntimeError("TorchScript output does not match the source model")
    path.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(path))
    loaded = torch.jit.load(str(path), map_location="cpu").eval()
    with torch.inference_mode():
        loaded_output = loaded(example)
    if tuple(loaded_output.shape) != (1, len(TARGET_CLASSES)):
        raise RuntimeError("saved TorchScript model failed its load smoke test")


def benchmark_torchscript(path: Path, image_size: int) -> float:
    import torch

    torch.set_num_threads(1)
    model = torch.jit.load(str(path), map_location="cpu").eval()
    example = torch.randn(1, 3, image_size, image_size)
    with torch.inference_mode():
        for _ in range(5):
            model(example)
        timings = []
        for _ in range(30):
            start = time.perf_counter()
            model(example)
            timings.append((time.perf_counter() - start) * 1000.0)
    return float(statistics.median(timings))


def quantize_model(model, calibration_loader, batches: int, image_size: int):
    import torch
    from torch.ao.quantization import get_default_qconfig_mapping
    from torch.ao.quantization.quantize_fx import convert_fx, prepare_fx

    torch.backends.quantized.engine = "qnnpack"
    model = copy.deepcopy(model).cpu().eval()
    example = torch.randn(1, 3, image_size, image_size)
    qconfig_mapping = get_default_qconfig_mapping("qnnpack")
    prepared = prepare_fx(
        model,
        qconfig_mapping,
        (example,),
    )
    observed_batches = 0
    observed_samples = 0
    with torch.inference_mode():
        for index, (images, _) in enumerate(calibration_loader):
            prepared(images.cpu())
            observed_batches += 1
            observed_samples += images.size(0)
            if index + 1 >= batches:
                break
    if observed_batches == 0:
        raise RuntimeError("INT8 calibration loader produced no images")
    quantized = convert_fx(prepared).eval()
    return quantized, {
        "mode": "post_training_static_fx",
        "backend": "qnnpack",
        "weight_dtype": "qint8",
        "activation_dtype": "quint8",
        "observer_batches": observed_batches,
        "observer_samples": observed_samples,
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def passes_export_gate(
    metrics: Metrics, min_macro_f1: float, min_class_recall: float
) -> tuple[bool, list[str]]:
    reasons: list[str] = []
    if metrics.macro_f1 < min_macro_f1:
        reasons.append(
            f"macro_f1 {metrics.macro_f1:.3f} < {min_macro_f1:.3f}"
        )
    weak = {
        label: recall
        for label, recall in metrics.per_class_recall.items()
        if recall < min_class_recall
    }
    if weak:
        reasons.append(
            "class recall below threshold: "
            + ", ".join(f"{label}={recall:.3f}" for label, recall in weak.items())
        )
    return not reasons, reasons


def main() -> None:
    args = parse_args()
    if args.image_size != 224:
        raise ValueError(
            "the current ROS node requires --image-size 224; change the runtime "
            "contract before training another size"
        )
    if args.min_per_class < 3:
        raise ValueError("--min-per-class must be at least 3")
    if args.recommended_per_class < args.min_per_class:
        raise ValueError(
            "--recommended-per-class must be at least --min-per-class"
        )
    if args.min_validation_per_class < 1:
        raise ValueError("--min-validation-per-class must be positive")
    if (
        args.min_per_class
        <= 2 * args.min_validation_per_class
    ):
        raise ValueError(
            "--min-per-class must leave training examples after validation "
            "and test minimums"
        )
    if args.head_epochs < 1 or args.finetune_epochs < 1:
        raise ValueError("training epoch counts must be positive")
    if args.early_stop_patience < 1 or args.lr_plateau_patience < 1:
        raise ValueError("scheduler and early-stop patience must be positive")
    if not 0.0 < args.lr_decay_factor < 1.0:
        raise ValueError("--lr-decay-factor must be between 0 and 1")
    if args.min_lr <= 0.0:
        raise ValueError("--min-lr must be positive")
    if args.rscd_download_workers < 1:
        raise ValueError("--rscd-download-workers must be positive")
    if not 0.0 <= args.model_selection_tolerance <= 1.0:
        raise ValueError("--model-selection-tolerance must be in [0, 1]")
    seed_everything(args.seed)
    raw_dir = args.work_dir / "raw"
    extracted_dir = args.work_dir / "extracted"
    raw_dir.mkdir(parents=True, exist_ok=True)
    extracted_dir.mkdir(parents=True, exist_ok=True)
    args.export_dir.mkdir(parents=True, exist_ok=True)
    if args.checkpoint_dir is not None:
        args.checkpoint_dir.mkdir(parents=True, exist_ok=True)

    prepared = None
    if args.dataset_manifest is not None:
        prepared = read_records(args.dataset_manifest)
        print(
            f"reusing explicit dataset manifest: {args.dataset_manifest} "
            f"({len(prepared[0])} images)",
            flush=True,
        )
        save_prepared_dataset(args.work_dir, args, prepared[0], prepared[1])
    else:
        prepared = load_prepared_dataset(args.work_dir, args)

    if prepared is None:
        collectors: list[tuple[str, Callable[[], list[Record]]]] = []
        if not args.skip_rscd:
            collectors.append(
                (
                    "RSCD",
                    lambda: collect_rscd(
                        extracted_dir,
                        args.rscd_per_class,
                        args.rscd_validation_per_class,
                        args.rscd_test_per_class,
                        args.seed,
                        args.rscd_download_workers,
                    ),
                )
            )
        if not args.skip_streetsurfacevis:
            collectors.append(
                (
                    "StreetSurfaceVis",
                    lambda: collect_streetsurfacevis(raw_dir, extracted_dir),
                )
            )
        if not args.skip_rtk:
            collectors.append(
                ("RTK Quality", lambda: collect_rtk(raw_dir, extracted_dir))
            )
        if not args.skip_cycling:
            collectors.append(
                ("Cycling", lambda: collect_cycling(raw_dir, extracted_dir))
            )
        if args.local_data_dir is not None:
            collectors.append(
                ("SafeStride local", lambda: collect_local(args.local_data_dir))
            )

        records = []
        source_errors: list[str] = []
        for name, collector in collectors:
            print(f"collecting {name}...", flush=True)
            try:
                collected = collector()
                print(f"{name}: {len(collected)} mapped images")
                records.extend(collected)
            except Exception as error:  # noqa: BLE001
                source_errors.append(f"{name}: {error!r}")
                print(f"{name} failed: {error!r}")
        if source_errors:
            print(
                "dataset source warning (coverage checks will decide whether "
                "training can continue):\n" + "\n".join(source_errors)
            )

        records = cap_records(
            records,
            args.max_per_source_class,
            args.max_per_class,
            args.seed,
        )
        records = validate_and_deduplicate(records)
        train_indices, validation_indices, test_indices = (
            stratified_group_split(records, args.seed)
        )
        split_indices = (train_indices, validation_indices, test_indices)
        save_prepared_dataset(args.work_dir, args, records, split_indices)
    else:
        records, split_indices = prepared
        train_indices, validation_indices, test_indices = split_indices

    require_class_coverage(
        records, args.min_per_class, args.recommended_per_class
    )
    print("source counts:", json.dumps(count_by_source(records), indent=2))
    require_split_coverage(
        records,
        (
            (
                "train",
                train_indices,
                args.min_per_class
                - 2 * args.min_validation_per_class,
            ),
            (
                "validation",
                validation_indices,
                args.min_validation_per_class,
            ),
            ("test", test_indices, args.min_validation_per_class),
        ),
    )
    split_by_index = {
        **{index: "train" for index in train_indices},
        **{index: "validation" for index in validation_indices},
        **{index: "test" for index in test_indices},
    }
    write_records(args.export_dir / "dataset_manifest.csv", records, split_by_index)
    prepared_dataset_fingerprint = dataset_fingerprint(records, split_indices)
    print(f"dataset fingerprint: {prepared_dataset_fingerprint}", flush=True)
    import torch

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"training device: {device}")
    results: list[CandidateResult] = []
    for model_name in args.models:
        seed_everything(args.seed)
        candidate_loaders = build_loaders(
            records,
            split_indices,
            args.image_size,
            args.batch_size,
            args.num_workers,
            device.type,
            args.seed,
        )
        results.append(
            train_candidate(
                model_name,
                candidate_loaders,
                args,
                device,
                prepared_dataset_fingerprint,
            )
        )
    selected = choose_candidate(results, args.model_selection_tolerance)

    loaders = build_loaders(
        records,
        split_indices,
        args.image_size,
        args.batch_size,
        args.num_workers,
        device.type,
        args.seed,
    )

    reports = {
        result.model_name: {
            "parameter_count": result.parameter_count,
            "validation": asdict(result.validation),
            "test": asdict(result.test),
            "history": result.history,
        }
        for result in results
    }
    (args.export_dir / "training_report.json").write_text(
        json.dumps(reports, indent=2), encoding="utf-8"
    )

    selected_model = build_model(
        selected.model_name, len(TARGET_CLASSES), pretrained=False
    )
    selected_model.load_state_dict(selected.state_dict)
    selected_model.eval()
    float_path = args.export_dir / "road_surface_float32_torchscript.pt"
    export_torchscript(selected_model, float_path, args.image_size)
    deployment_path = float_path
    deployment_validation_metrics = selected.validation
    deployment_metrics = selected.test
    quantization = "float32"
    quantization_report = {
        "requested": bool(args.quantize_int8),
        "accepted": False,
        "mode": "post_training_static_fx" if args.quantize_int8 else None,
        "backend": "qnnpack" if args.quantize_int8 else None,
        "weight_dtype": "qint8" if args.quantize_int8 else None,
        "activation_dtype": "quint8" if args.quantize_int8 else None,
        "calibration": None,
        "validation_macro_f1_drop": None,
        "validation_max_class_recall_drop": None,
        "error": None,
    }

    if args.quantize_int8:
        try:
            import torch.nn as nn

            calibration_loader, calibration_report = build_calibration_loader(
                records,
                train_indices,
                args.image_size,
                args.batch_size,
                args.num_workers,
                args.calibration_batches,
                args.seed,
            )
            quantized_model, observer_report = quantize_model(
                selected_model,
                calibration_loader,
                args.calibration_batches,
                args.image_size,
            )
            calibration_report.update(
                {
                    "requested_batches": args.calibration_batches,
                    "observer_batches": observer_report["observer_batches"],
                    "observer_samples": observer_report["observer_samples"],
                }
            )
            quantization_report.update(observer_report)
            quantization_report["calibration"] = calibration_report
            criterion = nn.CrossEntropyLoss(label_smoothing=args.label_smoothing)
            quantized_validation = run_epoch(
                quantized_model,
                loaders[1],
                criterion,
                torch.device("cpu"),
            )
            quantized_path = args.export_dir / "road_surface_int8_torchscript.pt"
            export_torchscript(quantized_model, quantized_path, args.image_size)
            macro_drop = (
                selected.validation.macro_f1 - quantized_validation.macro_f1
            )
            recall_drop = max(
                selected.validation.per_class_recall[label]
                - quantized_validation.per_class_recall[label]
                for label in TARGET_CLASSES
            )
            quantization_report[
                "validation_macro_f1_drop"
            ] = macro_drop
            quantization_report[
                "validation_max_class_recall_drop"
            ] = recall_drop
            if macro_drop <= 0.015 and recall_drop <= 0.05:
                quantized_test = run_epoch(
                    quantized_model,
                    loaders[2],
                    criterion,
                    torch.device("cpu"),
                )
                deployment_path = quantized_path
                deployment_validation_metrics = quantized_validation
                deployment_metrics = quantized_test
                quantization = "int8_qnnpack"
                quantization_report["accepted"] = True
            else:
                print(
                    "INT8 rejected: "
                    f"macro_f1_drop={macro_drop:.3f}, "
                    f"max_recall_drop={recall_drop:.3f}"
                )
        except Exception as error:  # noqa: BLE001
            quantization_report["error"] = repr(error)
            print(f"INT8 export unavailable; keeping float32: {error!r}")

    gate_ok, gate_reasons = passes_export_gate(
        deployment_metrics,
        args.min_macro_f1,
        args.min_class_recall,
    )
    artifact_name = (
        "road_surface_public_mix_torchscript.pt"
        if gate_ok
        else "road_surface_candidate_torchscript.pt"
    )
    final_path = args.export_dir / artifact_name
    shutil.copy2(deployment_path, final_path)
    classes_path = args.export_dir / "target_classes.json"
    classes_path.write_text(
        json.dumps(list(TARGET_CLASSES), indent=2) + "\n", encoding="utf-8"
    )
    manifest = {
        "schema_version": 1,
        "trained_at_utc": datetime.now(timezone.utc).isoformat(),
        "model_name": selected.model_name,
        "quantization": quantization,
        "quantization_report": quantization_report,
        "pruning": "none",
        "deployment_approved": gate_ok,
        "deployment_gate_reasons": gate_reasons,
        "input": {
            "layout": "NCHW",
            "dtype": "float32",
            "width": args.image_size,
            "height": args.image_size,
            "color": "RGB",
            "mean": IMAGENET_MEAN,
            "std": IMAGENET_STD,
        },
        "classes": TARGET_CLASSES,
        "output": "logits",
        "dataset": {
            "total": len(records),
            "class_counts": count_by_class(records),
            "source_counts": count_by_source(records),
            "split": {
                "train": len(train_indices),
                "validation": len(validation_indices),
                "test": len(test_indices),
            },
            "sources": {
                "rscd": {
                    "id": RSCD_REPO_ID,
                    "revision": (
                        (extracted_dir / "rscd_revision.txt")
                        .read_text(encoding="ascii")
                        .strip()
                        if (extracted_dir / "rscd_revision.txt").is_file()
                        else "not_used"
                    ),
                    "license": "MIT",
                },
                "streetsurfacevis": {
                    "record": STREETSURFACEVIS_RECORD,
                    "license": "CC-BY-SA",
                },
                "rtk": {
                    "dataset": RTK_DATASET_ID,
                    "version": 1,
                    "license": "CC-BY-4.0",
                },
                "cycling": {
                    "record": CYCLING_RECORD,
                    "license": "CC-BY-SA-4.0",
                },
            },
        },
        "training_config": {
            name: str(value) if isinstance(value, Path) else value
            for name, value in vars(args).items()
        },
        "metrics": {
            "validation": asdict(deployment_validation_metrics),
            "test": asdict(deployment_metrics),
        },
        "artifact": {
            "filename": final_path.name,
            "size_bytes": final_path.stat().st_size,
            "sha256": sha256_file(final_path),
            "median_cpu_latency_ms_colab": benchmark_torchscript(
                final_path, args.image_size
            ),
        },
        "runtime": {
            "torch": torch.__version__,
            "torchvision": __import__("torchvision").__version__,
        },
    }
    (args.export_dir / "model_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))
    if gate_ok:
        print(f"deployment artifacts are ready in {args.export_dir}")
    else:
        print(
            "training completed, but the candidate was not approved for ROS "
            "deployment:\n" + "\n".join(gate_reasons)
        )


if __name__ == "__main__":
    main()
