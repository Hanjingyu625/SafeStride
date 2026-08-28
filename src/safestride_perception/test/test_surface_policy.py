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


def test_deployed_model_labels_have_conservative_limits():
    expected = {
        'smooth_paved': 1.20,
        'rough_paved': 0.70,
        'block_paved': 0.65,
        'gravel': 0.55,
        'mud_dirt': 0.40,
        'unpaved_mixed': 0.50,
        'wet_paved': 0.50,
        'wet_unpaved': 0.40,
        'snow_ice': 0.0,
    }
    for label, scale in expected.items():
        assert speed_scale(label, 0.99) == scale
