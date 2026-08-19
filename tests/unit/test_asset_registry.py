from pathlib import Path
import hashlib
import pytest

from dexterous_robot.assets import AssetRegistryError, load_asset_registry


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_registry_resolves_device_first_asset_ids_and_verifies_hash(tmp_path: Path, monkeypatch):
    root = tmp_path / "robot_assets"
    wam = root / "arms/wam7/isaac/canonical_geometry_v2/controllers/wam.usda"
    hand = root / "hands/linker_l20/isaac/dynamic_v1/runtimes/l20.usda"
    wam.parent.mkdir(parents=True)
    hand.parent.mkdir(parents=True)
    wam.write_text("wam", encoding="utf-8")
    hand.write_text("l20", encoding="utf-8")
    monkeypatch.setenv("ROBOT_ASSETS_ROOT", str(root))

    root_cfg = tmp_path / "root.yaml"
    root_cfg.write_text("robot_assets_root: ${ROBOT_ASSETS_ROOT}\n", encoding="utf-8")
    manifest = tmp_path / "registry.yaml"
    manifest.write_text(
        "schema_version: 1\n"
        "assets:\n"
        "  arm.wam7.isaac.canonical_geometry_v2:\n"
        "    device_kind: arm\n"
        "    device_model: wam7\n"
        "    backend: isaac\n"
        "    version: canonical_geometry_v2\n"
        "    relative_path: arms/wam7/isaac/canonical_geometry_v2/controllers/wam.usda\n"
        f"    sha256: {_sha(wam)}\n"
        "    distribution: private_local\n"
        "  hand.linker_l20.isaac.dynamic_v1:\n"
        "    device_kind: hand\n"
        "    device_model: linker_l20\n"
        "    backend: isaac\n"
        "    version: dynamic_v1\n"
        "    relative_path: hands/linker_l20/isaac/dynamic_v1/runtimes/l20.usda\n"
        f"    sha256: {_sha(hand)}\n"
        "    distribution: private_local\n",
        encoding="utf-8",
    )
    registry = load_asset_registry(manifest, root_cfg)
    assert registry.resolve("arm.wam7.isaac.canonical_geometry_v2") == wam
    assert registry.resolve("hand.linker_l20.isaac.dynamic_v1") == hand


def test_registry_rejects_path_escape(tmp_path: Path):
    root_cfg = tmp_path / "root.yaml"
    root_cfg.write_text(f"robot_assets_root: {tmp_path / 'assets'}\n", encoding="utf-8")
    manifest = tmp_path / "registry.yaml"
    manifest.write_text(
        "schema_version: 1\nassets:\n"
        "  arm.wam7.isaac.bad:\n"
        "    device_kind: arm\n    device_model: wam7\n    backend: isaac\n    version: bad\n"
        "    relative_path: ../escape.usda\n"
        "    sha256: " + "a" * 64 + "\n"
        "    distribution: private_local\n",
        encoding="utf-8",
    )
    with pytest.raises(AssetRegistryError, match="ASSET_RELATIVE_PATH_INVALID"):
        load_asset_registry(manifest, root_cfg)


def test_registry_detects_hash_mismatch(tmp_path: Path):
    root = tmp_path / "assets"
    target = root / "arms/wam7/isaac/v/runtime.usda"
    target.parent.mkdir(parents=True)
    target.write_text("actual", encoding="utf-8")
    root_cfg = tmp_path / "root.yaml"
    root_cfg.write_text(f"robot_assets_root: {root}\n", encoding="utf-8")
    manifest = tmp_path / "registry.yaml"
    manifest.write_text(
        "schema_version: 1\nassets:\n"
        "  arm.wam7.isaac.v:\n"
        "    device_kind: arm\n    device_model: wam7\n    backend: isaac\n    version: v\n"
        "    relative_path: arms/wam7/isaac/v/runtime.usda\n"
        "    sha256: " + "f" * 64 + "\n"
        "    distribution: private_local\n",
        encoding="utf-8",
    )
    registry = load_asset_registry(manifest, root_cfg)
    with pytest.raises(AssetRegistryError, match="ASSET_SHA256_MISMATCH"):
        registry.resolve("arm.wam7.isaac.v")


def test_selection_resolves_roles_without_backend_specific_path_logic(tmp_path: Path, monkeypatch):
    from dexterous_robot.assets import load_asset_selection

    root = tmp_path / "assets"
    target = root / "arms/wam7/isaac/v/runtime.usda"
    target.parent.mkdir(parents=True)
    target.write_text("x", encoding="utf-8")
    monkeypatch.setenv("ROBOT_ASSETS_ROOT", str(root))
    root_cfg = tmp_path / "root.yaml"
    root_cfg.write_text("robot_assets_root: ${ROBOT_ASSETS_ROOT}\n", encoding="utf-8")
    manifest = tmp_path / "registry.yaml"
    manifest.write_text(
        "schema_version: 1\nassets:\n"
        "  arm.wam7.isaac.v:\n"
        "    device_kind: arm\n    device_model: wam7\n    backend: isaac\n    version: v\n"
        "    relative_path: arms/wam7/isaac/v/runtime.usda\n"
        f"    sha256: {_sha(target)}\n"
        "    distribution: private_local\n",
        encoding="utf-8",
    )
    selection_path = tmp_path / "selection.yaml"
    selection_path.write_text("schema_version: 1\nroles:\n  arm_runtime: arm.wam7.isaac.v\n", encoding="utf-8")
    registry = load_asset_registry(manifest, root_cfg)
    resolved = registry.resolve_selection(load_asset_selection(selection_path))
    assert resolved["arm_runtime"] == target
