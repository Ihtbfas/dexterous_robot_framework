from __future__ import annotations

import hashlib
import os
import re
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any, Mapping

import yaml

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_UNEXPANDED_ENV_RE = re.compile(r"\$(?:\{[^}]+\}|[A-Za-z_][A-Za-z0-9_]*)")
_ENTRY_KEYS = {
    "device_kind",
    "device_model",
    "backend",
    "version",
    "relative_path",
    "sha256",
    "distribution",
}


class AssetRegistryError(ValueError):
    """Raised when a robot-asset registry or resolved asset violates its contract."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path: str | Path, label: str) -> dict[str, Any]:
    p = Path(path)
    try:
        raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    except OSError as exc:
        raise AssetRegistryError(f"{label}_READ_FAILED:{p}") from exc
    except yaml.YAMLError as exc:
        raise AssetRegistryError(f"{label}_YAML_INVALID:{p}") from exc
    if not isinstance(raw, dict):
        raise AssetRegistryError(f"{label}_ROOT_INVALID")
    return raw


def _exact_keys(raw: Mapping[str, Any], expected: set[str], label: str) -> None:
    keys = set(raw)
    if keys != expected:
        missing = sorted(expected - keys)
        extra = sorted(keys - expected)
        raise AssetRegistryError(f"{label}_KEYS_INVALID:missing={missing}:extra={extra}")


def _expand_root(value: Any) -> Path:
    if not isinstance(value, str) or not value:
        raise AssetRegistryError("ROBOT_ASSETS_ROOT_INVALID")
    expanded = os.path.expandvars(value)
    if _UNEXPANDED_ENV_RE.search(expanded):
        raise AssetRegistryError(f"ROBOT_ASSETS_ROOT_ENV_UNEXPANDED:{expanded}")
    return Path(expanded).expanduser().resolve(strict=False)


def _relative_asset_path(value: Any) -> PurePosixPath:
    if not isinstance(value, str) or not value:
        raise AssetRegistryError("ASSET_RELATIVE_PATH_INVALID")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise AssetRegistryError(f"ASSET_RELATIVE_PATH_INVALID:{value}")
    return path


@dataclass(frozen=True)
class AssetEntry:
    asset_id: str
    device_kind: str
    device_model: str
    backend: str
    version: str
    relative_path: PurePosixPath
    sha256: str
    distribution: str


@dataclass(frozen=True)
class AssetSelection:
    roles: Mapping[str, str]


class AssetRegistry:
    __slots__ = ("_root", "_entries")

    def __init__(self, root: Path, entries: Mapping[str, AssetEntry]) -> None:
        self._root = root.resolve(strict=False)
        self._entries = MappingProxyType(dict(entries))

    @property
    def root(self) -> Path:
        return self._root

    @property
    def entries(self) -> Mapping[str, AssetEntry]:
        return self._entries

    def resolve(self, asset_id: str, *, verify_hash: bool = True) -> Path:
        try:
            entry = self._entries[asset_id]
        except KeyError as exc:
            raise AssetRegistryError(f"ASSET_ID_UNKNOWN:{asset_id}") from exc
        candidate = (self._root / Path(*entry.relative_path.parts)).resolve(strict=False)
        try:
            candidate.relative_to(self._root)
        except ValueError as exc:
            raise AssetRegistryError(f"ASSET_PATH_ESCAPES_ROOT:{asset_id}:{candidate}") from exc
        if not candidate.is_file():
            raise AssetRegistryError(f"ASSET_FILE_NOT_FOUND:{asset_id}:{candidate}")
        if verify_hash:
            actual = _sha256(candidate)
            if actual != entry.sha256:
                raise AssetRegistryError(
                    f"ASSET_SHA256_MISMATCH:{asset_id}:expected={entry.sha256}:actual={actual}"
                )
        return candidate

    def resolve_selection(self, selection: AssetSelection, *, verify_hash: bool = True) -> Mapping[str, Path]:
        return MappingProxyType(
            {role: self.resolve(asset_id, verify_hash=verify_hash) for role, asset_id in selection.roles.items()}
        )


def load_asset_registry(manifest_path: str | Path, root_config_path: str | Path) -> AssetRegistry:
    root_raw = _load_yaml(root_config_path, "ROBOT_ASSET_ROOT_CONFIG")
    _exact_keys(root_raw, {"robot_assets_root"}, "ROBOT_ASSET_ROOT_CONFIG")
    root = _expand_root(root_raw["robot_assets_root"])

    raw = _load_yaml(manifest_path, "ASSET_REGISTRY")
    _exact_keys(raw, {"schema_version", "assets"}, "ASSET_REGISTRY")
    if raw["schema_version"] != 1 or not isinstance(raw["assets"], dict) or not raw["assets"]:
        raise AssetRegistryError("ASSET_REGISTRY_SCHEMA_INVALID")

    entries: dict[str, AssetEntry] = {}
    for asset_id, entry_raw in raw["assets"].items():
        if not isinstance(asset_id, str) or not asset_id:
            raise AssetRegistryError("ASSET_ID_INVALID")
        if not isinstance(entry_raw, dict):
            raise AssetRegistryError(f"ASSET_ENTRY_INVALID:{asset_id}")
        _exact_keys(entry_raw, _ENTRY_KEYS, f"ASSET_ENTRY:{asset_id}")
        string_fields = ("device_kind", "device_model", "backend", "version", "distribution")
        if any(not isinstance(entry_raw[name], str) or not entry_raw[name] for name in string_fields):
            raise AssetRegistryError(f"ASSET_ENTRY_STRING_INVALID:{asset_id}")
        sha = entry_raw["sha256"]
        if not isinstance(sha, str) or not _SHA256_RE.fullmatch(sha):
            raise AssetRegistryError(f"ASSET_SHA256_INVALID:{asset_id}")
        parts = asset_id.split(".")
        if len(parts) < 4 or parts[0] != entry_raw["device_kind"] or parts[1] != entry_raw["device_model"] or parts[2] != entry_raw["backend"]:
            raise AssetRegistryError(f"ASSET_ID_METADATA_MISMATCH:{asset_id}")
        entries[asset_id] = AssetEntry(
            asset_id=asset_id,
            device_kind=entry_raw["device_kind"],
            device_model=entry_raw["device_model"],
            backend=entry_raw["backend"],
            version=entry_raw["version"],
            relative_path=_relative_asset_path(entry_raw["relative_path"]),
            sha256=sha,
            distribution=entry_raw["distribution"],
        )
    return AssetRegistry(root, entries)


def load_asset_selection(path: str | Path) -> AssetSelection:
    raw = _load_yaml(path, "ASSET_SELECTION")
    _exact_keys(raw, {"schema_version", "roles"}, "ASSET_SELECTION")
    if raw["schema_version"] != 1 or not isinstance(raw["roles"], dict) or not raw["roles"]:
        raise AssetRegistryError("ASSET_SELECTION_SCHEMA_INVALID")
    roles: dict[str, str] = {}
    for role, asset_id in raw["roles"].items():
        if not isinstance(role, str) or not role or not isinstance(asset_id, str) or not asset_id:
            raise AssetRegistryError("ASSET_SELECTION_ROLE_INVALID")
        roles[role] = asset_id
    return AssetSelection(MappingProxyType(roles))
