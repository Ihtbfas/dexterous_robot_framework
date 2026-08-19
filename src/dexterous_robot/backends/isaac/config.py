from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

# Compatibility import for backend.py/scene.py type annotations only. Task schema
# ownership and parsing live in dexterous_robot.config.tasks.
from dexterous_robot.config.tasks import TabletopGraspLiftConfig


class IsaacConfigError(ValueError):
    """Raised when a tracked Isaac backend YAML violates its exact schema."""


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
    hand_grasp_lock: HandDriveProfile21


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
