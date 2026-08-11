from dataclasses import replace
from safestride_terrain.deployment_policy import DeploymentInputs, may_deploy


def test_every_interlock_is_required():
    safe = DeploymentInputs(True, True, True, 0.0, True, 0)
    assert may_deploy(safe)
    assert not may_deploy(replace(safe, tof_valid=False))
    assert not may_deploy(replace(safe, attitude_valid=False))
    assert not may_deploy(replace(safe, both_hands_present=False))
    assert not may_deploy(replace(safe, wheel_speed_mps=0.1))
    assert not may_deploy(replace(safe, retracted_limit=False))
    assert not may_deploy(replace(safe, fault_bits=1))
