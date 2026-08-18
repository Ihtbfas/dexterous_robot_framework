from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any

import yaml

from .models import LocalAssetConfig

_EXPECTED_LOCAL_ASSET_KEYS = {"wam_runtime", "l20_runtime"}
_UNEXPANDED_ENV_PATTERN = re.compile(r"\$(?:\{[^}]+\}|[A-Za-z_][A-Za-z0-9_]*)")


class ConfigError(ValueError):
    """Raised when a framework configuration file violates its exact schema."""


def _expanded_path(value: Any, *, key: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ConfigError(f"LOCAL_ASSET_CONFIG_PATH_INVALID:{key}")
    expanded = os.path.expandvars(value)
    if _UNEXPANDED_ENV_PATTERN.search(expanded):
        raise ConfigError(f"LOCAL_ASSET_CONFIG_ENV_UNEXPANDED:{key}:{expanded}")
    return Path(expanded).expanduser()


def load_local_asset_config(path: str | Path) -> LocalAssetConfig:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigError(f"LOCAL_ASSET_CONFIG_READ_FAILED:{config_path}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"LOCAL_ASSET_CONFIG_YAML_INVALID:{config_path}") from exc

    if not isinstance(raw, dict):
        raise ConfigError("LOCAL_ASSET_CONFIG_ROOT_INVALID")
    keys = set(raw)
    if keys != _EXPECTED_LOCAL_ASSET_KEYS:
        missing = sorted(_EXPECTED_LOCAL_ASSET_KEYS - keys)
        extra = sorted(keys - _EXPECTED_LOCAL_ASSET_KEYS)
        raise ConfigError(f"LOCAL_ASSET_CONFIG_KEYS_INVALID:missing={missing}:extra={extra}")

    return LocalAssetConfig(
        wam_runtime=_expanded_path(raw["wam_runtime"], key="wam_runtime"),
        l20_runtime=_expanded_path(raw["l20_runtime"], key="l20_runtime"),
    )
