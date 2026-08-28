"""Tests for road-surface dataset mapping and deployment gates."""

from __future__ import annotations

from pathlib import Path
from contextlib import redirect_stdout
import io
import json
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_DIR))

from road_surface_labels import (  # noqa: E402
    TARGET_CLASSES,
    map_cycling_label,
    map_rscd_label,
    map_rtk_quality_label,
    map_streetsurfacevis,
)
from train_road_surface import (  # noqa: E402
    CandidateResult,
    Metrics,
    Record,
    choose_candidate,
    collect_rscd,
    dataset_fingerprint,
    dataset_split_hint,
    passes_export_gate,
    read_records,
    require_class_coverage,
    select_calibration_indices,
    write_records,
)


class RoadSurfaceLabelTest(unittest.TestCase):
    def test_ros_class_order_is_stable(self) -> None:
        self.assertEqual(
            TARGET_CLASSES,
            (
                "smooth_paved",
                "rough_paved",
                "block_paved",
                "gravel",
                "mud_dirt",
                "unpaved_mixed",
                "wet_paved",
                "wet_unpaved",
                "snow_ice",
            ),
        )

    def test_rscd_mapping(self) -> None:
        cases = {
            "dry-asphalt-smooth": "smooth_paved",
            "asphalt-severe": "rough_paved",
            "wet-concrete": "wet_paved",
            "water-gravel": "wet_unpaved",
            "fresh-snow": "snow_ice",
            "cobblestone": "block_paved",
            "dry-gravel": "gravel",
            "mud": "mud_dirt",
        }
        for raw, expected in cases.items():
            with self.subTest(raw=raw):
                self.assertEqual(map_rscd_label(raw), expected)

    def test_other_public_dataset_mappings(self) -> None:
        self.assertEqual(
            map_streetsurfacevis("asphalt", "good"), "smooth_paved"
        )
        self.assertEqual(
            map_streetsurfacevis("asphalt", "bad"), "rough_paved"
        )
        self.assertEqual(
            map_streetsurfacevis("paving_stones", "good"), "block_paved"
        )
        self.assertEqual(
            map_streetsurfacevis("unpaved", "bad"), "unpaved_mixed"
        )
        self.assertEqual(map_rtk_quality_label("asphalt_good"), "smooth_paved")
        self.assertEqual(map_rtk_quality_label("unpaved_bad"), "unpaved_mixed")
        self.assertEqual(map_cycling_label("paving_stones"), "block_paved")


class DeploymentGateTest(unittest.TestCase):
    def metrics(self, macro_f1: float, weak_recall: float = 0.8) -> Metrics:
        recalls = {label: 0.8 for label in TARGET_CLASSES}
        recalls["snow_ice"] = weak_recall
        return Metrics(
            loss=0.2,
            accuracy=0.8,
            macro_f1=macro_f1,
            per_class_precision={label: 0.8 for label in TARGET_CLASSES},
            per_class_recall=recalls,
            per_class_f1={label: 0.8 for label in TARGET_CLASSES},
            support={label: 30 for label in TARGET_CLASSES},
            confusion_matrix=[[0] * len(TARGET_CLASSES) for _ in TARGET_CLASSES],
        )

    def test_gate_accepts_balanced_model(self) -> None:
        passed, reasons = passes_export_gate(self.metrics(0.8), 0.7, 0.45)
        self.assertTrue(passed)
        self.assertEqual(reasons, [])

    def test_gate_rejects_weak_class_even_with_good_macro_f1(self) -> None:
        passed, reasons = passes_export_gate(
            self.metrics(0.8, weak_recall=0.2), 0.7, 0.45
        )
        self.assertFalse(passed)
        self.assertIn("snow_ice=0.200", reasons[0])

    def test_official_boolean_split_is_parsed_without_string_truthiness(self) -> None:
        self.assertEqual(dataset_split_hint(True), "train")
        self.assertEqual(dataset_split_hint("False"), "test")
        self.assertEqual(dataset_split_hint("unknown"), "")

    def test_model_selection_prefers_size_only_within_f1_tolerance(self) -> None:
        small = CandidateResult(
            "small", 2_000_000, {}, self.metrics(0.80), self.metrics(0.80), []
        )
        large = CandidateResult(
            "large", 5_000_000, {}, self.metrics(0.806), self.metrics(0.80), []
        )
        self.assertIs(choose_candidate((small, large), 0.005), large)
        self.assertIs(choose_candidate((small, large), 0.01), small)


class DataCoverageTest(unittest.TestCase):
    @staticmethod
    def records(per_class: int) -> list[Record]:
        return [
            Record(
                path=f"{label}-{index}.jpg",
                label=label,
                source="test",
                raw_label=label,
                group=f"{label}-{index}",
            )
            for label in TARGET_CLASSES
            for index in range(per_class)
        ]

    def test_recommended_shortage_warns_but_does_not_abort(self) -> None:
        output = io.StringIO()
        with redirect_stdout(output):
            require_class_coverage(self.records(60), 60, 250)
        self.assertIn("training will continue", output.getvalue())

    def test_hard_floor_still_protects_unmeasurable_class(self) -> None:
        with redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                require_class_coverage(self.records(59), 60, 250)

    def test_calibration_subset_is_balanced_and_deterministic(self) -> None:
        records = self.records(20)
        indices = list(range(len(records)))
        first = select_calibration_indices(records, indices, 45, seed=42)
        second = select_calibration_indices(records, indices, 45, seed=42)

        self.assertEqual(first, second)
        self.assertEqual(len(first), 45)
        counts = {
            label: sum(records[index].label == label for index in first)
            for label in TARGET_CLASSES
        }
        self.assertEqual(set(counts.values()), {5})

    def test_calibration_subset_never_uses_non_train_records(self) -> None:
        records = self.records(3)
        train_indices = list(range(0, len(records), 2))
        selected = select_calibration_indices(
            records, train_indices, 100, seed=7
        )
        self.assertTrue(set(selected).issubset(train_indices))

    def test_prepared_manifest_round_trip_preserves_splits(self) -> None:
        records = [
            Record(
                path=f"{label}.jpg",
                label=label,
                source="test",
                raw_label=label,
                group=label,
                sha256=str(index),
            )
            for index, label in enumerate(TARGET_CLASSES[:3])
        ]
        splits = ([0], [1], [2])
        split_by_index = {0: "train", 1: "validation", 2: "test"}
        with tempfile.TemporaryDirectory() as temporary:
            manifest = Path(temporary) / "dataset_manifest.csv"
            write_records(manifest, records, split_by_index)
            loaded_records, loaded_splits = read_records(manifest)

        self.assertEqual(loaded_records, records)
        self.assertEqual(loaded_splits, splits)
        self.assertEqual(
            dataset_fingerprint(loaded_records, loaded_splits),
            dataset_fingerprint(records, splits),
        )


class NotebookConfigurationTest(unittest.TestCase):
    def test_colab_defaults_to_one_resumable_model(self) -> None:
        notebook_path = (
            Path(__file__).resolve().parents[1]
            / "notebooks"
            / "road_surface_training_colab.ipynb"
        )
        notebook = json.loads(notebook_path.read_text(encoding="utf-8"))
        source = "\n".join(
            "".join(cell.get("source", [])) for cell in notebook["cells"]
        )
        self.assertIn("'--resume'", source)
        self.assertIn("'--checkpoint-dir'", source)
        self.assertIn("'--models', 'mobilenet_v3_small'", source)
        self.assertNotIn(
            "'--models', 'mobilenet_v3_small', 'mobilenet_v3_large'", source
        )


class RscdCollectionTest(unittest.TestCase):
    def test_indexing_stops_after_each_class_quota_is_met(self) -> None:
        class FakeEntry:
            def __init__(self, path: str) -> None:
                self.path = path

        class FakeApi:
            scanned = 0

            @staticmethod
            def dataset_info(repo_id: str):
                return types.SimpleNamespace(sha="test-revision")

            def list_repo_tree(self, path_in_repo: str, **_kwargs):
                if path_in_repo.startswith("train/"):
                    raw = path_in_repo.split("/", 1)[1]
                    names = [f"{path_in_repo}/{raw}-{index}.jpg" for index in range(100)]
                else:
                    raw_labels = (
                        "dry_asphalt_smooth",
                        "dry_asphalt_severe",
                        "dry_gravel",
                        "dry_mud",
                        "wet_asphalt_smooth",
                        "wet_gravel",
                        "ice",
                    )
                    names = [
                        f"{path_in_repo}/{index}-{raw.replace('_', '-')}.jpg"
                        for index, raw in enumerate(raw_labels)
                    ]
                for name in names:
                    self.scanned += 1
                    yield FakeEntry(name)

        api = FakeApi()
        huggingface = types.ModuleType("huggingface_hub")
        huggingface.HfApi = lambda: api
        huggingface.hf_hub_download = (
            lambda filename, local_dir, **_kwargs: str(Path(local_dir) / filename)
        )
        tqdm_package = types.ModuleType("tqdm")
        tqdm_auto = types.ModuleType("tqdm.auto")
        tqdm_auto.tqdm = lambda iterable, **_kwargs: iterable

        modules = {
            "huggingface_hub": huggingface,
            "tqdm": tqdm_package,
            "tqdm.auto": tqdm_auto,
        }
        with tempfile.TemporaryDirectory() as temporary:
            with patch.dict(sys.modules, modules):
                records = collect_rscd(
                    Path(temporary),
                    train_per_class=2,
                    validation_per_class=1,
                    test_per_class=1,
                    seed=42,
                    download_workers=2,
                )

        self.assertEqual(len(records), 28)
        self.assertEqual(api.scanned, 28)


if __name__ == "__main__":
    unittest.main()
