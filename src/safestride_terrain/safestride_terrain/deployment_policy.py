from dataclasses import dataclass


@dataclass(frozen=True)
class DeploymentInputs:
    tof_valid: bool
    attitude_valid: bool
    both_hands_present: bool
    wheel_speed_mps: float
    retracted_limit: bool
    fault_bits: int


def may_deploy(value: DeploymentInputs, max_speed_mps: float = 0.03) -> bool:
    return (value.tof_valid and value.attitude_valid and
            value.both_hands_present and value.retracted_limit and
            value.fault_bits == 0 and
            abs(value.wheel_speed_mps) <= max_speed_mps)
