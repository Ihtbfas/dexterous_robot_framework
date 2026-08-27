from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class MuJoCoConfigError(ValueError):
    """Raised when a tracked MuJoCo backend YAML violates its exact schema."""


def _exact_keys(raw: Any, expected: set[str], label: str) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise MuJoCoConfigError(f"{label}_ROOT_INVALID")
    keys = set(raw)
    if keys != expected:
        missing = sorted(expected - keys)
        extra = sorted(keys - expected)
        raise MuJoCoConfigError(
            f"{label}_KEYS_INVALID:missing={missing}:extra={extra}"
        )
    return raw


def _load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise MuJoCoConfigError(f"MUJOCO_CONFIG_READ_FAILED:{config_path}") from exc
    except yaml.YAMLError as exc:
        raise MuJoCoConfigError(f"MUJOCO_CONFIG_YAML_INVALID:{config_path}") from exc
    if not isinstance(raw, dict):
        raise MuJoCoConfigError("MUJOCO_CONFIG_ROOT_INVALID")
    return raw


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise MuJoCoConfigError(f"{label}_INVALID")
    return value


def _positive_finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MuJoCoConfigError(f"{label}_INVALID") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise MuJoCoConfigError(f"{label}_INVALID")
    return result


@dataclass(frozen=True)
class MuJoCoModelConfig:
    arm_asset_role: str
    hand_asset_role: str


@dataclass(frozen=True)
class MuJoCoFrameConfig:
    wam_base: str
    wam_flange: str
    l20_base: str


@dataclass(frozen=True)
class MuJoCoViewerConfig:
    enabled: bool


@dataclass(frozen=True)
class MuJoCoBackendConfig:
    physics_timestep_s: float
    runtime_dt_tolerance_s: float
    model: MuJoCoModelConfig
    frames: MuJoCoFrameConfig
    viewer: MuJoCoViewerConfig


def load_mujoco_backend_config(path: str | Path) -> MuJoCoBackendConfig:
    raw = _load_yaml(path)
    _exact_keys(
        raw,
        {
            "schema_version",
            "kind",
            "physics_timestep_s",
            "runtime_dt_tolerance_s",
            "model",
            "frames",
            "viewer",
        },
        "MUJOCO_BACKEND_CONFIG",
    )
    if raw["schema_version"] != 1 or raw["kind"] != "MuJoCoBackend":
        raise MuJoCoConfigError("MUJOCO_BACKEND_CONFIG_SCHEMA_INVALID")

    model_raw = _exact_keys(
        raw["model"],
        {"arm_asset_role", "hand_asset_role"},
        "MUJOCO_MODEL_CONFIG",
    )
    frames_raw = _exact_keys(
        raw["frames"],
        {"wam_base", "wam_flange", "l20_base"},
        "MUJOCO_FRAME_CONFIG",
    )
    viewer_raw = _exact_keys(
        raw["viewer"],
        {"enabled"},
        "MUJOCO_VIEWER_CONFIG",
    )
    if type(viewer_raw["enabled"]) is not bool:
        raise MuJoCoConfigError("MUJOCO_VIEWER_ENABLED_INVALID")

    return MuJoCoBackendConfig(
        physics_timestep_s=_positive_finite(
            raw["physics_timestep_s"], "MUJOCO_PHYSICS_TIMESTEP"
        ),
        runtime_dt_tolerance_s=_positive_finite(
            raw["runtime_dt_tolerance_s"], "MUJOCO_RUNTIME_DT_TOLERANCE"
        ),
        model=MuJoCoModelConfig(
            arm_asset_role=_nonempty_string(
                model_raw["arm_asset_role"], "MUJOCO_ARM_ASSET_ROLE"
            ),
            hand_asset_role=_nonempty_string(
                model_raw["hand_asset_role"], "MUJOCO_HAND_ASSET_ROLE"
            ),
        ),
        frames=MuJoCoFrameConfig(
            wam_base=_nonempty_string(frames_raw["wam_base"], "MUJOCO_WAM_BASE"),
            wam_flange=_nonempty_string(
                frames_raw["wam_flange"], "MUJOCO_WAM_FLANGE"
            ),
            l20_base=_nonempty_string(frames_raw["l20_base"], "MUJOCO_L20_BASE"),
        ),
        viewer=MuJoCoViewerConfig(enabled=viewer_raw["enabled"]),
    )
