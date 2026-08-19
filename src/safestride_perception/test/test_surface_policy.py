from safestride_perception.surface_policy import speed_scale


def test_unknown_and_low_confidence_stop():
    assert speed_scale('unknown', 0.99) == 0.0
    assert speed_scale('smooth', 0.2) == 0.0


def test_hazard_never_requests_motion():
    assert speed_scale('step', 0.99) == 0.0
    assert speed_scale('hole', 0.99) == 0.0


def test_deployed_model_labels_have_conservative_limits():
    assert speed_scale('smooth_paved', 0.99) == 1.0
    assert speed_scale('rough_paved', 0.99) == 0.55
    assert speed_scale('wet_unpaved', 0.99) == 0.30
    assert speed_scale('snow_ice', 0.99) == 0.0
