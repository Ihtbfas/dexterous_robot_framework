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
    profiles_raw = _exact_keys(raw["profiles"], {"arm_carry_position_drive", "hand_open_hold"}, "ISAAC_PROFILES")
    arm_raw = _exact_keys(profiles_raw["arm_carry_position_drive"], {"stiffness", "damping", "max_force"}, "ISAAC_ARM_CARRY_PROFILE")
    hand_raw = _exact_keys(
        profiles_raw["hand_open_hold"],
        {"stiffness_usd_per_degree", "damping_usd_per_degree_per_second", "max_force_nm", "max_joint_velocity_deg_s", "drive_type"},
        "ISAAC_HAND_OPEN_HOLD_PROFILE",
    )

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

    profile = DriveProfile7(
        stiffness=_float_tuple(arm_raw["stiffness"], 7, "ISAAC_ARM_STIFFNESS"),
        damping=_float_tuple(arm_raw["damping"], 7, "ISAAC_ARM_DAMPING"),
        max_force=_float_tuple(arm_raw["max_force"], 7, "ISAAC_ARM_MAX_FORCE"),
    )
    if hand_raw["drive_type"] != "force":
        raise IsaacConfigError("ISAAC_HAND_OPEN_HOLD_DRIVE_TYPE_INVALID")
    hand_profile = HandDriveProfile21(
        stiffness_usd_per_degree=_float_tuple(hand_raw["stiffness_usd_per_degree"], 21, "ISAAC_HAND_STIFFNESS"),
        damping_usd_per_degree_per_second=_float_tuple(hand_raw["damping_usd_per_degree_per_second"], 21, "ISAAC_HAND_DAMPING"),
        max_force_nm=_float_tuple(hand_raw["max_force_nm"], 21, "ISAAC_HAND_MAX_FORCE"),
        max_joint_velocity_deg_s=_float_tuple(hand_raw["max_joint_velocity_deg_s"], 21, "ISAAC_HAND_MAX_JOINT_VELOCITY"),
        drive_type=str(hand_raw["drive_type"]),
    )
    return IsaacBackendConfig(
        physics_dt_s=float(raw["physics_dt_s"]),
        stage_load_timeout_s=float(raw["stage_load_timeout_s"]),
        asset_authority=IsaacAssetAuthority(**asset_raw),
        paths=IsaacPaths(**paths_raw),
        transform_sync=sync,
        arm_carry_position_drive=profile,
        hand_open_hold=hand_profile,
    )


def load_tabletop_grasp_lift_config(path: str | Path) -> TabletopGraspLiftConfig:
    raw = _load_yaml(path)
    _exact_keys(raw, {"schema_version", "kind", "world_frame", "table_top_world_z_m", "table", "object", "initialization"}, "TABLETOP_TASK_CONFIG")
    if raw["schema_version"] != 1 or raw["kind"] != "TabletopGraspLiftTask":
        raise IsaacConfigError("TABLETOP_TASK_CONFIG_SCHEMA_INVALID")
    table = _exact_keys(raw["table"], {"center_world_m", "dimensions_xyz_m", "yaw_rad"}, "TABLETOP_TABLE")
    obj = _exact_keys(raw["object"], {"center_world_m", "dimensions_xyz_m", "yaw_rad", "mass_kg", "static_friction", "dynamic_friction", "restitution"}, "TABLETOP_OBJECT")
    init = _exact_keys(raw["initialization"], {"wam_q_rad", "hand_q_rad", "l20_root_position_world_m", "l20_root_quaternion_xyzw"}, "TABLETOP_INITIALIZATION")
    return TabletopGraspLiftConfig(
        world_frame=str(raw["world_frame"]),
        table_top_world_z_m=float(raw["table_top_world_z_m"]),
        table_center_world_m=_float_tuple(table["center_world_m"], 3, "TABLE_CENTER"),  # type: ignore[arg-type]
        table_dimensions_xyz_m=_float_tuple(table["dimensions_xyz_m"], 3, "TABLE_DIMENSIONS"),  # type: ignore[arg-type]
        table_yaw_rad=float(table["yaw_rad"]),
        object_position_world_m=_float_tuple(obj["center_world_m"], 3, "OBJECT_CENTER"),  # type: ignore[arg-type]
        object_dimensions_xyz_m=_float_tuple(obj["dimensions_xyz_m"], 3, "OBJECT_DIMENSIONS"),  # type: ignore[arg-type]
        object_yaw_rad=float(obj["yaw_rad"]),
        object_mass_kg=float(obj["mass_kg"]),
        object_static_friction=float(obj["static_friction"]),
        object_dynamic_friction=float(obj["dynamic_friction"]),
        object_restitution=float(obj["restitution"]),
        initial_wam_q_rad=_float_tuple(init["wam_q_rad"], 7, "INITIAL_WAM_Q"),
        initial_hand_q_rad=_float_tuple(init["hand_q_rad"], 21, "INITIAL_HAND_Q"),
        initial_l20_root_position_world_m=_float_tuple(init["l20_root_position_world_m"], 3, "INITIAL_L20_POSITION"),  # type: ignore[arg-type]
        initial_l20_root_quaternion_xyzw=_float_tuple(init["l20_root_quaternion_xyzw"], 4, "INITIAL_L20_QUATERNION"),  # type: ignore[arg-type]
    )
