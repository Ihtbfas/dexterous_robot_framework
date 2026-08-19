from pathlib import Path
import yaml


def test_tracked_registry_is_project_independent_and_device_first() -> None:
    root = Path(__file__).resolve().parents[2]
    registry_path = root / "configs/assets/registry.yaml"
    raw = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    assert raw["schema_version"] == 1
    assets = raw["assets"]
    assert set(assets) == {
        "arm.wam7.isaac.canonical_geometry_v2",
        "hand.linker_l20.isaac.dynamic_v1",
    }
    paths = [entry["relative_path"] for entry in assets.values()]
    assert paths[0].startswith("arms/wam7/")
    assert paths[1].startswith("hands/linker_l20/")
    text = registry_path.read_text(encoding="utf-8").lower()
    assert "phase2b0" not in text
    assert "/home/lyf" not in text
    assert "dexterous_robot_framework" not in text


def test_asset_root_example_uses_shared_robot_assets_root() -> None:
    root = Path(__file__).resolve().parents[2]
    path = root / "configs/assets/robot_assets.example.yaml"
    assert path.read_text(encoding="utf-8").strip() == "robot_assets_root: ${ROBOT_ASSETS_ROOT}"


def test_legacy_direct_path_asset_sample_is_removed() -> None:
    root = Path(__file__).resolve().parents[2]
    assert not (root / "configs/assets/local_assets.example.yaml").exists()
