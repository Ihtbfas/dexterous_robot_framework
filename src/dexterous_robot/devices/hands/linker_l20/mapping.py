from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType

from .types import (
    L20_PHYSICAL_JOINTS,
    L20ActiveCommand16,
    L20PhysicalTarget21,
)


@dataclass(frozen=True)
class L20JointSpec:
    name: str
    lower_rad: float
    upper_rad: float
    active_channel: str
    coupled_from: str | None = None


L20_JOINT_SPECS: tuple[L20JointSpec, ...] = (
    L20JointSpec("thumb_joint0", -0.297, 0.683, "thumb_roll"),
    L20JointSpec("thumb_joint1", 0.0, 1.78, "thumb_yaw"),
    L20JointSpec("thumb_joint2", 0.0, 0.87, "thumb_root_flex"),
    L20JointSpec("thumb_joint3", 0.0, 1.29, "thumb_tip_flex"),
    L20JointSpec("thumb_joint4", 0.0, 1.29, "thumb_tip_flex", "thumb_joint3"),
    L20JointSpec("index_joint0", -0.26, 0.26, "index_yaw"),
    L20JointSpec("index_joint1", 0.0, 1.4, "index_root_flex"),
    L20JointSpec("index_joint2", 0.0, 1.08, "index_tip_flex"),
    L20JointSpec("index_joint3", 0.0, 1.15, "index_tip_flex", "index_joint2"),
    L20JointSpec("middle_joint0", -0.26, 0.26, "middle_yaw"),
    L20JointSpec("middle_joint1", 0.0, 1.4, "middle_root_flex"),
    L20JointSpec("middle_joint2", 0.0, 1.08, "middle_tip_flex"),
    L20JointSpec("middle_joint3", 0.0, 1.15, "middle_tip_flex", "middle_joint2"),
    L20JointSpec("ring_joint0", -0.26, 0.26, "ring_yaw"),
    L20JointSpec("ring_joint1", 0.0, 1.4, "ring_root_flex"),
    L20JointSpec("ring_joint2", 0.0, 1.08, "ring_tip_flex"),
    L20JointSpec("ring_joint3", 0.0, 1.15, "ring_tip_flex", "ring_joint2"),
    L20JointSpec("little_joint0", -0.26, 0.26, "little_yaw"),
    L20JointSpec("little_joint1", 0.0, 1.4, "little_root_flex"),
    L20JointSpec("little_joint2", 0.0, 1.08, "little_tip_flex"),
    L20JointSpec("little_joint3", 0.0, 1.15, "little_tip_flex", "little_joint2"),
)

if tuple(spec.name for spec in L20_JOINT_SPECS) != L20_PHYSICAL_JOINTS:
    raise RuntimeError("L20_JOINT_SPEC_ORDER_INVALID")

COUPLING_MULTIPLIERS = MappingProxyType(
    {
        "urdf_mimic_v1": MappingProxyType(
            {
                "thumb_joint4": 1.0,
                "index_joint3": 1.06399,
                "middle_joint3": 1.06399,
                "ring_joint3": 1.06399,
                "little_joint3": 1.06399,
            }
        ),
        "mujoco_equal_v1": MappingProxyType(
            {
                "thumb_joint4": 1.0,
                "index_joint3": 1.0,
                "middle_joint3": 1.0,
                "ring_joint3": 1.0,
                "little_joint3": 1.0,
            }
        ),
    }
)

SUPPORTED_COUPLING_PROFILES: tuple[str, ...] = tuple(COUPLING_MULTIPLIERS)


def map_active_to_physical(
    command: L20ActiveCommand16,
    *,
    coupling_profile: str,
) -> L20PhysicalTarget21:
    """Map normalized Active16 to Physical21 radians with only semantic coupling.

    Target-rate limiting intentionally does not live here; it belongs to a
    controller/runtime policy rather than the device-space definition.
    """
    if not isinstance(command, L20ActiveCommand16):
        raise ValueError("L20_ACTIVE_MAPPING_INVALID")
    multipliers = COUPLING_MULTIPLIERS.get(coupling_profile)
    if multipliers is None:
        raise ValueError("L20_COUPLING_PROFILE_INVALID")

    active_values = command.as_mapping()
    leader_targets: dict[str, float] = {}
    for spec in L20_JOINT_SPECS:
        if spec.coupled_from is None:
            leader_targets[spec.name] = spec.lower_rad + active_values[spec.active_channel] * (spec.upper_rad - spec.lower_rad)

    physical: list[float] = []
    for spec in L20_JOINT_SPECS:
        if spec.coupled_from is None:
            value = leader_targets[spec.name]
        else:
            value = leader_targets[spec.coupled_from] * multipliers[spec.name]
        physical.append(min(spec.upper_rad, max(spec.lower_rad, value)))

    return L20PhysicalTarget21(
        tuple(physical),
        coupling_profile,
        source_timestamp_s=command.timestamp_s,
        sequence_id=command.sequence_id,
    )
