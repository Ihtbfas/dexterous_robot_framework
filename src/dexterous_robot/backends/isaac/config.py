from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class IsaacConfigError(ValueError):
    """Raised when a tracked Isaac/backend task YAML violates its exact schema."""


def _exact_keys(raw: Any, expected: set[str], label: str) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise IsaacConfigError(f"{label}_ROOT_INVALID")
    keys = set(raw)
    if keys != expected:
        missing = sorted(expected - keys)
        extra = sorted(keys - expected)
        raise IsaacConfigError(f"{label}_KEYS_INVALID:missing={missing}:extra={extra}")
    return raw


def _load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise IsaacConfigError(f"ISAAC_CONFIG_READ_FAILED:{config_path}") from exc
    except yaml.YAMLError as exc:
        raise IsaacConfigError(f"ISAAC_CONFIG_YAML_INVALID:{config_path}") from exc
    if not isinstance(raw, dict):
        raise IsaacConfigError("ISAAC_CONFIG_ROOT_INVALID")
    return raw


def _float_tuple(value: Any, width: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != width:
        raise IsaacConfigError(f"{label}_INVALID")
    try:
        return tuple(float(x) for x in value)
    except (TypeError, ValueError) as exc:
        raise IsaacConfigError(f"{label}_INVALID") from exc


def _positive(value: Any, label: str, *, allow_zero: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise IsaacConfigError(f"{label}_INVALID") from exc
    if result < 0.0 or (result == 0.0 and not allow_zero):
        raise IsaacConfigError(f"{label}_INVALID")
    return result


@dataclass(frozen=True)
class DriveProfile7:
    stiffness: tuple[float, ...]
    damping: tuple[float, ...]
    max_force: tuple[float, ...]


@dataclass(frozen=True)
class HandDriveProfile21:
    stiffness_usd_per_degree: tuple[float, ...]
    damping_usd_per_degree_per_second: tuple[float, ...]
    max_force_nm: tuple[float, ...]
    max_joint_velocity_deg_s: tuple[float, ...]
    drive_type: str


@dataclass(frozen=True)
class TransformSyncConfig:
    update_to_fast_cache: bool
    update_to_usd: bool
    position_tolerance_m: float


@dataclass(frozen=True)
class IsaacPaths:
    l20_root: str
    l20_world_fixed_joint: str
    integration_scope: str
    integration_fixed_joint: str
    wam_j7_body: str
    l20_base_body: str
    table: str
    object: str
    grasp_material: str
    scene_material: str


@dataclass(frozen=True)
class IsaacAssetAuthority:
    wam_runtime_sha256: str
    l20_runtime_sha256: str


@dataclass(frozen=True)
class IsaacBackendConfig:
    physics_dt_s: float
    stage_load_timeout_s: float
    asset_authority: IsaacAssetAuthority
    paths: IsaacPaths
    transform_sync: TransformSyncConfig
    arm_carry_position_drive: DriveProfile7
    hand_open_hold: HandDriveProfile21
    hand_grasp_lock: HandDriveProfile21


@dataclass(frozen=True)
class ApproachControlConfig:
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
class TabletopGraspLiftConfig:
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


def _hand_profile(raw: Any, label: str) -> HandDriveProfile21:
    hand_raw = _exact_keys(
        raw,
        {"stiffness_usd_per_degree", "damping_usd_per_degree_per_second", "max_force_nm", "max_joint_velocity_deg_s", "drive_type"},
        label,
    )
    if hand_raw["drive_type"] != "force":
        raise IsaacConfigError(f"{label}_DRIVE_TYPE_INVALID")
    return HandDriveProfile21(
        stiffness_usd_per_degree=_float_tuple(hand_raw["stiffness_usd_per_degree"], 21, f"{label}_STIFFNESS"),
        damping_usd_per_degree_per_second=_float_tuple(hand_raw["damping_usd_per_degree_per_second"], 21, f"{label}_DAMPING"),
        max_force_nm=_float_tuple(hand_raw["max_force_nm"], 21, f"{label}_MAX_FORCE"),
        max_joint_velocity_deg_s=_float_tuple(hand_raw["max_joint_velocity_deg_s"], 21, f"{label}_MAX_JOINT_VELOCITY"),
        drive_type="force",
    )


def load_isaac_backend_config(path: str | Path) -> IsaacBackendConfig:
    raw = _load_yaml(path)
    _exact_keys(
        raw,
        {"schema_version", "kind", "physics_dt_s", "stage_load_timeout_s", "asset_authority", "paths", "transform_sync", "profiles"},
        "ISAAC_BACKEND_CONFIG",
    )
    if raw["schema_version"] != 1 or raw["kind"] != "IsaacBackend":
        raise IsaacConfigError("ISAAC_BACKEND_CONFIG_SCHEMA_INVALID")

    asset_raw = _exact_keys(raw["asset_authority"], {"wam_runtime_sha256", "l20_runtime_sha256"}, "ISAAC_ASSET_AUTHORITY")
    path_keys = {
        "l20_root", "l20_world_fixed_joint", "integration_scope", "integration_fixed_joint",
        "wam_j7_body", "l20_base_body", "table", "object", "grasp_material", "scene_material",
    }
    paths_raw = _exact_keys(raw["paths"], path_keys, "ISAAC_PATHS")
    sync_raw = _exact_keys(raw["transform_sync"], {"update_to_fast_cache", "update_to_usd", "position_tolerance_m"}, "ISAAC_TRANSFORM_SYNC")
    profiles_raw = _exact_keys(raw["profiles"], {"arm_carry_position_drive", "hand_open_hold", "hand_grasp_lock"}, "ISAAC_PROFILES")
    arm_raw = _exact_keys(profiles_raw["arm_carry_position_drive"], {"stiffness", "damping", "max_force"}, "ISAAC_ARM_CARRY_PROFILE")

    for key, value in paths_raw.items():
        if not isinstance(value, str) or not value.startswith("/"):
            raise IsaacConfigError(f"ISAAC_PATH_INVALID:{key}")
    for key, value in asset_raw.items():
        if not isinstance(value, str) or len(value) != 64:
            raise IsaacConfigError(f"ISAAC_ASSET_SHA_INVALID:{key}")

    sync = TransformSyncConfig(
        update_to_fast_cache=bool(sync_raw["update_to_fast_cache"]),
        update_to_usd=bool(sync_raw["update_to_usd"]),
        position_tolerance_m=float(sync_raw["position_tolerance_m"]),
    )
    if sync.position_tolerance_m <= 0.0:
        raise IsaacConfigError("ISAAC_TRANSFORM_SYNC_NUMERIC_INVALID")

    arm_profile = DriveProfile7(
        stiffness=_float_tuple(arm_raw["stiffness"], 7, "ISAAC_ARM_STIFFNESS"),
        damping=_float_tuple(arm_raw["damping"], 7, "ISAAC_ARM_DAMPING"),
        max_force=_float_tuple(arm_raw["max_force"], 7, "ISAAC_ARM_MAX_FORCE"),
    )
    return IsaacBackendConfig(
        physics_dt_s=float(raw["physics_dt_s"]),
        stage_load_timeout_s=float(raw["stage_load_timeout_s"]),
        asset_authority=IsaacAssetAuthority(**asset_raw),
        paths=IsaacPaths(**paths_raw),
        transform_sync=sync,
        arm_carry_position_drive=arm_profile,
        hand_open_hold=_hand_profile(profiles_raw["hand_open_hold"], "ISAAC_HAND_OPEN_HOLD_PROFILE"),
        hand_grasp_lock=_hand_profile(profiles_raw["hand_grasp_lock"], "ISAAC_HAND_GRASP_LOCK_PROFILE"),
    )


def load_tabletop_grasp_lift_config(path: str | Path) -> TabletopGraspLiftConfig:
    raw = _load_yaml(path)
    _exact_keys(raw, {"schema_version", "kind", "world_frame", "table_top_world_z_m", "table", "object", "initialization", "control"}, "TABLETOP_TASK_CONFIG")
    if raw["schema_version"] != 1 or raw["kind"] != "TabletopGraspLiftTask":
        raise IsaacConfigError("TABLETOP_TASK_CONFIG_SCHEMA_INVALID")
    table = _exact_keys(raw["table"], {"center_world_m", "dimensions_xyz_m", "yaw_rad"}, "TABLETOP_TABLE")
    obj = _exact_keys(raw["object"], {"center_world_m", "dimensions_xyz_m", "yaw_rad", "mass_kg", "static_friction", "dynamic_friction", "restitution"}, "TABLETOP_OBJECT")
    init = _exact_keys(raw["initialization"], {"wam_q_rad", "hand_q_rad", "l20_root_position_world_m", "l20_root_quaternion_xyzw"}, "TABLETOP_INITIALIZATION")
    control = _exact_keys(raw["control"], {"approach", "grasp", "lift", "hold"}, "TABLETOP_CONTROL")
    approach = _exact_keys(control["approach"], {"lateral_ready_q_rad", "transit_q_rad", "pregrasp_q_rad", "grasp_q_rad", "preshape_hand_q_rad", "waypoint_duration_s", "preshape_duration_s", "settle_duration_s", "joint_tolerance_rad"}, "TABLETOP_APPROACH")
    grasp = _exact_keys(control["grasp"], {"base_preload_hand_q_rad", "release_settle_s", "preload_duration_s", "lock_ramp_duration_s", "lock_hold_duration_s", "target_squeeze_n"}, "TABLETOP_GRASP")
    lift = _exact_keys(control["lift"], {"delta_world_z_m", "duration_s", "minimum_object_rise_m", "max_table_normal_n", "max_relative_drift_m"}, "TABLETOP_LIFT")
    hold = _exact_keys(control["hold"], {"duration_s", "minimum_clearance_m", "max_table_normal_n", "max_relative_drift_m"}, "TABLETOP_HOLD")

    return TabletopGraspLiftConfig(
        world_frame=str(raw["world_frame"]),
        table_top_world_z_m=float(raw["table_top_world_z_m"]),
        table_center_world_m=_float_tuple(table["center_world_m"], 3, "TABLE_CENTER"),
        table_dimensions_xyz_m=_float_tuple(table["dimensions_xyz_m"], 3, "TABLE_DIMENSIONS"),
        table_yaw_rad=float(table["yaw_rad"]),
        object_position_world_m=_float_tuple(obj["center_world_m"], 3, "OBJECT_CENTER"),
        object_dimensions_xyz_m=_float_tuple(obj["dimensions_xyz_m"], 3, "OBJECT_DIMENSIONS"),
        object_yaw_rad=float(obj["yaw_rad"]),
        object_mass_kg=float(obj["mass_kg"]),
        object_static_friction=float(obj["static_friction"]),
        object_dynamic_friction=float(obj["dynamic_friction"]),
        object_restitution=float(obj["restitution"]),
        initial_wam_q_rad=_float_tuple(init["wam_q_rad"], 7, "INITIAL_WAM_Q"),
        initial_hand_q_rad=_float_tuple(init["hand_q_rad"], 21, "INITIAL_HAND_Q"),
        initial_l20_root_position_world_m=_float_tuple(init["l20_root_position_world_m"], 3, "INITIAL_L20_POSITION"),
        initial_l20_root_quaternion_xyzw=_float_tuple(init["l20_root_quaternion_xyzw"], 4, "INITIAL_L20_QUATERNION"),
        control=TabletopControlConfig(
            approach=ApproachControlConfig(
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
            grasp=GraspControlConfig(
                base_preload_hand_q_rad=_float_tuple(grasp["base_preload_hand_q_rad"], 21, "GRASP_BASE_PRELOAD_Q"),
                release_settle_s=_positive(grasp["release_settle_s"], "GRASP_RELEASE_SETTLE", allow_zero=True),
                preload_duration_s=_positive(grasp["preload_duration_s"], "GRASP_PRELOAD_DURATION"),
                lock_ramp_duration_s=_positive(grasp["lock_ramp_duration_s"], "GRASP_LOCK_RAMP_DURATION"),
                lock_hold_duration_s=_positive(grasp["lock_hold_duration_s"], "GRASP_LOCK_HOLD_DURATION"),
                target_squeeze_n=_positive(grasp["target_squeeze_n"], "GRASP_TARGET_SQUEEZE", allow_zero=True),
            ),
            lift=LiftControlConfig(
                delta_world_z_m=_positive(lift["delta_world_z_m"], "LIFT_DELTA_Z"),
                duration_s=_positive(lift["duration_s"], "LIFT_DURATION"),
                minimum_object_rise_m=_positive(lift["minimum_object_rise_m"], "LIFT_MINIMUM_RISE", allow_zero=True),
                max_table_normal_n=_positive(lift["max_table_normal_n"], "LIFT_MAX_TABLE_NORMAL", allow_zero=True),
                max_relative_drift_m=_positive(lift["max_relative_drift_m"], "LIFT_MAX_RELATIVE_DRIFT"),
            ),
            hold=HoldControlConfig(
                duration_s=_positive(hold["duration_s"], "HOLD_DURATION"),
                minimum_clearance_m=_positive(hold["minimum_clearance_m"], "HOLD_MINIMUM_CLEARANCE", allow_zero=True),
                max_table_normal_n=_positive(hold["max_table_normal_n"], "HOLD_MAX_TABLE_NORMAL", allow_zero=True),
                max_relative_drift_m=_positive(hold["max_relative_drift_m"], "HOLD_MAX_RELATIVE_DRIFT"),
            ),
        ),
    )
