from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any

import yaml

from dexterous_robot.motion.profiles import MotionProfiles


class TaskConfigError(ValueError):
    """Raised when a backend-neutral task document violates its exact schema."""


def _exact_keys(raw: Any, expected: set[str], label: str) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise TaskConfigError(f"{label}_ROOT_INVALID")
    keys = set(raw)
    if keys != expected:
        missing = sorted(expected - keys)
        extra = sorted(keys - expected)
        raise TaskConfigError(f"{label}_KEYS_INVALID:missing={missing}:extra={extra}")
    return raw


def _load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise TaskConfigError(f"TASK_CONFIG_READ_FAILED:{config_path}") from exc
    except yaml.YAMLError as exc:
        raise TaskConfigError(f"TASK_CONFIG_YAML_INVALID:{config_path}") from exc
    if not isinstance(raw, dict):
        raise TaskConfigError("TASK_CONFIG_ROOT_INVALID")
    return raw


def _float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise TaskConfigError(f"{label}_INVALID")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TaskConfigError(f"{label}_INVALID") from exc
    if not isfinite(result):
        raise TaskConfigError(f"{label}_INVALID")
    return result


def _float_tuple(value: Any, width: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != width:
        raise TaskConfigError(f"{label}_INVALID")
    return tuple(_float(v, label) for v in value)


def _positive(value: Any, label: str, *, allow_zero: bool = False) -> float:
    result = _float(value, label)
    if result < 0.0 or (result == 0.0 and not allow_zero):
        raise TaskConfigError(f"{label}_INVALID")
    return result


def _name(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise TaskConfigError(f"{label}_INVALID")
    return value


@dataclass(frozen=True)
class ApproachControlConfig:
    lateral_ready_q_rad: tuple[float, ...]
    transit_q_rad: tuple[float, ...]
    pregrasp_q_rad: tuple[float, ...]
    grasp_q_rad: tuple[float, ...]
    preshape_hand_q_rad: tuple[float, ...]
    motion_profile: str
    preshape_duration_s: float
    settle_duration_s: float
    joint_tolerance_rad: float


@dataclass(frozen=True)
class LegacyApproachControlConfigV1:
    lateral_ready_q_rad: tuple[float, ...]
    transit_q_rad: tuple[float, ...]
    pregrasp_q_rad: tuple[float, ...]
    grasp_q_rad: tuple[float, ...]
    preshape_hand_q_rad: tuple[float, ...]
    waypoint_duration_s: float
    preshape_duration_s: float
    settle_duration_s: float
    joint_tolerance_rad: float


@dataclass(frozen=True)
class GraspControlConfig:
    base_preload_hand_q_rad: tuple[float, ...]
    release_settle_s: float
    preload_duration_s: float
    lock_ramp_duration_s: float
    lock_hold_duration_s: float
    target_squeeze_n: float


@dataclass(frozen=True)
class LiftControlConfig:
    delta_world_z_m: float
    motion_profile: str
    minimum_object_rise_m: float
    max_table_normal_n: float
    max_relative_drift_m: float


@dataclass(frozen=True)
class LegacyLiftControlConfigV1:
    delta_world_z_m: float
    duration_s: float
    minimum_object_rise_m: float
    max_table_normal_n: float
    max_relative_drift_m: float


@dataclass(frozen=True)
class HoldControlConfig:
    duration_s: float
    minimum_clearance_m: float
    max_table_normal_n: float
    max_relative_drift_m: float


@dataclass(frozen=True)
class TabletopControlConfig:
    approach: ApproachControlConfig
    grasp: GraspControlConfig
    lift: LiftControlConfig
    hold: HoldControlConfig


@dataclass(frozen=True)
class LegacyTabletopControlConfigV1:
    approach: LegacyApproachControlConfigV1
    grasp: GraspControlConfig
    lift: LegacyLiftControlConfigV1
    hold: HoldControlConfig


@dataclass(frozen=True)
class TabletopGraspLiftConfig:
    schema_version: int
    world_frame: str
    table_top_world_z_m: float
    table_center_world_m: tuple[float, float, float]
    table_dimensions_xyz_m: tuple[float, float, float]
    table_yaw_rad: float
    object_position_world_m: tuple[float, float, float]
    object_dimensions_xyz_m: tuple[float, float, float]
    object_yaw_rad: float
    object_mass_kg: float
    object_static_friction: float
    object_dynamic_friction: float
    object_restitution: float
    initial_wam_q_rad: tuple[float, ...]
    initial_hand_q_rad: tuple[float, ...]
    initial_l20_root_position_world_m: tuple[float, float, float]
    initial_l20_root_quaternion_xyzw: tuple[float, float, float, float]
    control: TabletopControlConfig


@dataclass(frozen=True)
class LegacyTabletopGraspLiftConfigV1:
    schema_version: int
    world_frame: str
    table_top_world_z_m: float
    table_center_world_m: tuple[float, float, float]
    table_dimensions_xyz_m: tuple[float, float, float]
    table_yaw_rad: float
    object_position_world_m: tuple[float, float, float]
    object_dimensions_xyz_m: tuple[float, float, float]
    object_yaw_rad: float
    object_mass_kg: float
    object_static_friction: float
    object_dynamic_friction: float
    object_restitution: float
    initial_wam_q_rad: tuple[float, ...]
    initial_hand_q_rad: tuple[float, ...]
    initial_l20_root_position_world_m: tuple[float, float, float]
    initial_l20_root_quaternion_xyzw: tuple[float, float, float, float]
    control: LegacyTabletopControlConfigV1


def _parse_common(raw: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    _exact_keys(raw, {"schema_version", "kind", "world_frame", "table_top_world_z_m", "table", "object", "initialization", "control"}, "TABLETOP_TASK_CONFIG")
    if raw["kind"] != "TabletopGraspLiftTask":
        raise TaskConfigError("TASK_CONFIG_KIND_INVALID")
    table = _exact_keys(raw["table"], {"center_world_m", "dimensions_xyz_m", "yaw_rad"}, "TABLETOP_TABLE")
    obj = _exact_keys(raw["object"], {"center_world_m", "dimensions_xyz_m", "yaw_rad", "mass_kg", "static_friction", "dynamic_friction", "restitution"}, "TABLETOP_OBJECT")
    init = _exact_keys(raw["initialization"], {"wam_q_rad", "hand_q_rad", "l20_root_position_world_m", "l20_root_quaternion_xyzw"}, "TABLETOP_INITIALIZATION")
    control = _exact_keys(raw["control"], {"approach", "grasp", "lift", "hold"}, "TABLETOP_CONTROL")
    common = dict(
        world_frame=_name(raw["world_frame"], "WORLD_FRAME"),
        table_top_world_z_m=_float(raw["table_top_world_z_m"], "TABLE_TOP_WORLD_Z"),
        table_center_world_m=_float_tuple(table["center_world_m"], 3, "TABLE_CENTER"),
        table_dimensions_xyz_m=_float_tuple(table["dimensions_xyz_m"], 3, "TABLE_DIMENSIONS"),
        table_yaw_rad=_float(table["yaw_rad"], "TABLE_YAW"),
        object_position_world_m=_float_tuple(obj["center_world_m"], 3, "OBJECT_CENTER"),
        object_dimensions_xyz_m=_float_tuple(obj["dimensions_xyz_m"], 3, "OBJECT_DIMENSIONS"),
        object_yaw_rad=_float(obj["yaw_rad"], "OBJECT_YAW"),
        object_mass_kg=_positive(obj["mass_kg"], "OBJECT_MASS"),
        object_static_friction=_positive(obj["static_friction"], "OBJECT_STATIC_FRICTION", allow_zero=True),
        object_dynamic_friction=_positive(obj["dynamic_friction"], "OBJECT_DYNAMIC_FRICTION", allow_zero=True),
        object_restitution=_positive(obj["restitution"], "OBJECT_RESTITUTION", allow_zero=True),
        initial_wam_q_rad=_float_tuple(init["wam_q_rad"], 7, "INITIAL_WAM_Q"),
        initial_hand_q_rad=_float_tuple(init["hand_q_rad"], 21, "INITIAL_HAND_Q"),
        initial_l20_root_position_world_m=_float_tuple(init["l20_root_position_world_m"], 3, "INITIAL_L20_POSITION"),
        initial_l20_root_quaternion_xyzw=_float_tuple(init["l20_root_quaternion_xyzw"], 4, "INITIAL_L20_QUATERNION"),
    )
    return common, control


def _grasp(raw: Any) -> GraspControlConfig:
    grasp = _exact_keys(raw, {"base_preload_hand_q_rad", "release_settle_s", "preload_duration_s", "lock_ramp_duration_s", "lock_hold_duration_s", "target_squeeze_n"}, "TABLETOP_GRASP")
    return GraspControlConfig(
        base_preload_hand_q_rad=_float_tuple(grasp["base_preload_hand_q_rad"], 21, "GRASP_BASE_PRELOAD_Q"),
        release_settle_s=_positive(grasp["release_settle_s"], "GRASP_RELEASE_SETTLE", allow_zero=True),
        preload_duration_s=_positive(grasp["preload_duration_s"], "GRASP_PRELOAD_DURATION"),
        lock_ramp_duration_s=_positive(grasp["lock_ramp_duration_s"], "GRASP_LOCK_RAMP_DURATION"),
        lock_hold_duration_s=_positive(grasp["lock_hold_duration_s"], "GRASP_LOCK_HOLD_DURATION"),
        target_squeeze_n=_positive(grasp["target_squeeze_n"], "GRASP_TARGET_SQUEEZE", allow_zero=True),
    )


def _hold(raw: Any) -> HoldControlConfig:
    hold = _exact_keys(raw, {"duration_s", "minimum_clearance_m", "max_table_normal_n", "max_relative_drift_m"}, "TABLETOP_HOLD")
    return HoldControlConfig(
        duration_s=_positive(hold["duration_s"], "HOLD_DURATION"),
        minimum_clearance_m=_positive(hold["minimum_clearance_m"], "HOLD_MINIMUM_CLEARANCE", allow_zero=True),
        max_table_normal_n=_positive(hold["max_table_normal_n"], "HOLD_MAX_TABLE_NORMAL", allow_zero=True),
        max_relative_drift_m=_positive(hold["max_relative_drift_m"], "HOLD_MAX_RELATIVE_DRIFT"),
    )


def _parse_v1(raw: dict[str, Any]) -> LegacyTabletopGraspLiftConfigV1:
    common, control = _parse_common(raw)
    approach = _exact_keys(control["approach"], {"lateral_ready_q_rad", "transit_q_rad", "pregrasp_q_rad", "grasp_q_rad", "preshape_hand_q_rad", "waypoint_duration_s", "preshape_duration_s", "settle_duration_s", "joint_tolerance_rad"}, "TABLETOP_APPROACH")
    lift = _exact_keys(control["lift"], {"delta_world_z_m", "duration_s", "minimum_object_rise_m", "max_table_normal_n", "max_relative_drift_m"}, "TABLETOP_LIFT")
    return LegacyTabletopGraspLiftConfigV1(
        schema_version=1,
        **common,
        control=LegacyTabletopControlConfigV1(
            approach=LegacyApproachControlConfigV1(
                lateral_ready_q_rad=_float_tuple(approach["lateral_ready_q_rad"], 7, "APPROACH_LATERAL_READY_Q"),
                transit_q_rad=_float_tuple(approach["transit_q_rad"], 7, "APPROACH_TRANSIT_Q"),
                pregrasp_q_rad=_float_tuple(approach["pregrasp_q_rad"], 7, "APPROACH_PREGRASP_Q"),
                grasp_q_rad=_float_tuple(approach["grasp_q_rad"], 7, "APPROACH_GRASP_Q"),
                preshape_hand_q_rad=_float_tuple(approach["preshape_hand_q_rad"], 21, "APPROACH_PRESHAPE_HAND_Q"),
                waypoint_duration_s=_positive(approach["waypoint_duration_s"], "APPROACH_WAYPOINT_DURATION"),
                preshape_duration_s=_positive(approach["preshape_duration_s"], "APPROACH_PRESHAPE_DURATION"),
                settle_duration_s=_positive(approach["settle_duration_s"], "APPROACH_SETTLE_DURATION", allow_zero=True),
                joint_tolerance_rad=_positive(approach["joint_tolerance_rad"], "APPROACH_JOINT_TOLERANCE"),
            ),
            grasp=_grasp(control["grasp"]),
            lift=LegacyLiftControlConfigV1(
                delta_world_z_m=_positive(lift["delta_world_z_m"], "LIFT_DELTA_Z"),
                duration_s=_positive(lift["duration_s"], "LIFT_DURATION"),
                minimum_object_rise_m=_positive(lift["minimum_object_rise_m"], "LIFT_MINIMUM_RISE", allow_zero=True),
                max_table_normal_n=_positive(lift["max_table_normal_n"], "LIFT_MAX_TABLE_NORMAL", allow_zero=True),
                max_relative_drift_m=_positive(lift["max_relative_drift_m"], "LIFT_MAX_RELATIVE_DRIFT"),
            ),
            hold=_hold(control["hold"]),
        ),
    )


def _parse_v2(raw: dict[str, Any]) -> TabletopGraspLiftConfig:
    common, control = _parse_common(raw)
    approach = _exact_keys(control["approach"], {"lateral_ready_q_rad", "transit_q_rad", "pregrasp_q_rad", "grasp_q_rad", "preshape_hand_q_rad", "motion_profile", "preshape_duration_s", "settle_duration_s", "joint_tolerance_rad"}, "TABLETOP_APPROACH")
    lift = _exact_keys(control["lift"], {"delta_world_z_m", "motion_profile", "minimum_object_rise_m", "max_table_normal_n", "max_relative_drift_m"}, "TABLETOP_LIFT")
    return TabletopGraspLiftConfig(
        schema_version=2,
        **common,
        control=TabletopControlConfig(
            approach=ApproachControlConfig(
                lateral_ready_q_rad=_float_tuple(approach["lateral_ready_q_rad"], 7, "APPROACH_LATERAL_READY_Q"),
                transit_q_rad=_float_tuple(approach["transit_q_rad"], 7, "APPROACH_TRANSIT_Q"),
                pregrasp_q_rad=_float_tuple(approach["pregrasp_q_rad"], 7, "APPROACH_PREGRASP_Q"),
                grasp_q_rad=_float_tuple(approach["grasp_q_rad"], 7, "APPROACH_GRASP_Q"),
                preshape_hand_q_rad=_float_tuple(approach["preshape_hand_q_rad"], 21, "APPROACH_PRESHAPE_HAND_Q"),
                motion_profile=_name(approach["motion_profile"], "APPROACH_MOTION_PROFILE"),
                preshape_duration_s=_positive(approach["preshape_duration_s"], "APPROACH_PRESHAPE_DURATION"),
                settle_duration_s=_positive(approach["settle_duration_s"], "APPROACH_SETTLE_DURATION", allow_zero=True),
                joint_tolerance_rad=_positive(approach["joint_tolerance_rad"], "APPROACH_JOINT_TOLERANCE"),
            ),
            grasp=_grasp(control["grasp"]),
            lift=LiftControlConfig(
                delta_world_z_m=_positive(lift["delta_world_z_m"], "LIFT_DELTA_Z"),
                motion_profile=_name(lift["motion_profile"], "LIFT_MOTION_PROFILE"),
                minimum_object_rise_m=_positive(lift["minimum_object_rise_m"], "LIFT_MINIMUM_RISE", allow_zero=True),
                max_table_normal_n=_positive(lift["max_table_normal_n"], "LIFT_MAX_TABLE_NORMAL", allow_zero=True),
                max_relative_drift_m=_positive(lift["max_relative_drift_m"], "LIFT_MAX_RELATIVE_DRIFT"),
            ),
            hold=_hold(control["hold"]),
        ),
    )


def load_tabletop_grasp_lift_document(path: str | Path) -> TabletopGraspLiftConfig | LegacyTabletopGraspLiftConfigV1:
    raw = _load_yaml(path)
    version = raw.get("schema_version")
    if version == 1:
        return _parse_v1(raw)
    if version == 2:
        return _parse_v2(raw)
    raise TaskConfigError("TASK_CONFIG_SCHEMA_INVALID")


def load_tabletop_grasp_lift_config(path: str | Path, *, motion_profiles: MotionProfiles) -> TabletopGraspLiftConfig:
    if not isinstance(motion_profiles, MotionProfiles):
        raise TaskConfigError("TASK_CONFIG_MOTION_PROFILES_INVALID")
    doc = load_tabletop_grasp_lift_document(path)
    if not isinstance(doc, TabletopGraspLiftConfig):
        raise TaskConfigError("TASK_CONFIG_CURRENT_SCHEMA_REQUIRED")
    motion_profiles.joint(doc.control.approach.motion_profile)
    motion_profiles.cartesian(doc.control.lift.motion_profile)
    return doc
