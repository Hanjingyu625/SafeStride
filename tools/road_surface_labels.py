"""Shared label contract for public road-surface training data."""

from __future__ import annotations

import re


TARGET_CLASSES = (
    "smooth_paved",
    "rough_paved",
    "block_paved",
    "gravel",
    "mud_dirt",
    "unpaved_mixed",
    "wet_paved",
    "wet_unpaved",
    "snow_ice",
)


def clean_text(value: object) -> str:
    """Normalize labels from the public datasets."""

    text = str(value).strip().lower()
    return re.sub(r"[^a-z0-9]+", "_", text).strip("_")


def map_rscd_label(raw: object) -> str | None:
    """Map RSCD's condition-material-severity labels to SafeStride."""

    label = clean_text(raw)
    tokens = set(label.split("_"))
    if tokens & {"ice", "snow", "slush"}:
        return "snow_ice"

    is_wet = bool(tokens & {"wet", "water", "puddle", "flooded"})
    is_unpaved = bool(tokens & {"gravel", "mud", "dirt", "earth"})
    if is_wet:
        return "wet_unpaved" if is_unpaved else "wet_paved"
    if "gravel" in tokens:
        return "gravel"
    if tokens & {"mud", "dirt", "earth"}:
        return "mud_dirt"
    if tokens & {"cobblestone", "paver", "paving", "sett", "brick"}:
        return "block_paved"
    if tokens & {"asphalt", "concrete"}:
        if tokens & {"smooth", "good", "excellent"}:
            return "smooth_paved"
        return "rough_paved"
    return None


def map_streetsurfacevis(
    surface_type: object,
    surface_quality: object,
) -> str | None:
    """Map StreetSurfaceVis type and quality labels."""

    surface = clean_text(surface_type)
    quality = clean_text(surface_quality)
    if surface in {"paving_stones", "paving_stone", "sett"}:
        return "block_paved"
    if surface == "unpaved":
        return "unpaved_mixed"
    if surface in {"asphalt", "concrete"}:
        if quality in {"excellent", "good", "1", "2", "1_0", "2_0"}:
            return "smooth_paved"
        return "rough_paved"
    return None


def map_rtk_quality_label(raw: object) -> str | None:
    """Map the RTK surface-quality directory name."""

    label = clean_text(raw)
    if label.startswith("asphalt_good"):
        return "smooth_paved"
    if label.startswith("asphalt_"):
        return "rough_paved"
    if label.startswith("paved_"):
        return "block_paved"
    if label.startswith("unpaved_"):
        return "unpaved_mixed"
    return None


def map_cycling_label(raw: object) -> str | None:
    """Map the edge-oriented cycling dataset labels."""

    label = clean_text(raw)
    if "asphalt" in label:
        return "smooth_paved"
    if "paving" in label or "sett" in label:
        return "block_paved"
    if "unpaved" in label:
        return "unpaved_mixed"
    return None
