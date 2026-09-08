from safestride_perception.surface_policy import (
    prediction_is_confident,
    speed_scale,
)


def test_prediction_requires_confidence_and_clear_top1_margin():
    assert prediction_is_confident(0.80, 0.30)
    assert not prediction_is_confident(0.70, 0.62)
    assert not prediction_is_confident(0.60, 0.20)
    assert not prediction_is_confident(float('nan'), 0.20)


def test_unknown_and_low_confidence_stop():
    assert speed_scale('unknown', 0.99) == 0.0
    assert speed_scale('smooth', 0.2) == 0.0


def test_hazard_never_requests_motion():
    assert speed_scale('step', 0.99) == 0.0
    assert speed_scale('hole', 0.99) == 0.0


def test_deployed_model_labels_have_driveable_limits():
    expected = {
        'smooth_paved': 1.00,
        'rough_paved': 1.00,
        'block_paved': 0.95,
        'gravel': 0.95,
        'mud_dirt': 1.00,
        'unpaved_mixed': 1.00,
        'wet_paved': 0.85,
        'wet_unpaved': 0.85,
        'snow_ice': 0.75,
    }
    for label, scale in expected.items():
        assert speed_scale(label, 0.99) == scale


def test_dry_paved_confusion_does_not_change_speed():
    assert speed_scale('smooth_paved', 0.99) == speed_scale(
        'rough_paved', 0.99
    )


def test_traversable_surfaces_keep_a_usable_velocity_target():
    for label in (
        'smooth_paved',
        'rough_paved',
        'block_paved',
        'gravel',
        'mud_dirt',
        'unpaved_mixed',
        'wet_paved',
        'wet_unpaved',
        'snow_ice',
    ):
        assert 0.75 <= speed_scale(label, 0.99) <= 1.00
