from pathlib import Path

import yaml


REQUIRED_ASSET_IDS = {
    "arm.wam7.isaac.canonical_geometry_v2",
    "hand.linker_l20.isaac.dynamic_v1",
    "arm.wam7.mujoco.canonical_geometry_v2",
    "hand.linker_l20.mujoco.right_v1",
}


def test_tracked_registry_is_project_independent_and_device_first() -> None:
    root = Path(__file__).resolve().parents[2]
    registry_path = root / "configs/assets/registry.yaml"
    raw = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    assert raw["schema_version"] == 1
    assets = raw["assets"]

    # The registry is intentionally extensible across backends. Adding MuJoCo
    # assets must not invalidate the device-first/project-independent contract.
    assert REQUIRED_ASSET_IDS <= set(assets)

    for asset_id, entry in assets.items():
        path = entry["relative_path"]
        assert not path.startswith("/"), asset_id

        if entry["device_kind"] == "arm":
            assert path.startswith(f"arms/{entry['device_model']}/"), asset_id
        elif entry["device_kind"] == "hand":
            assert path.startswith(f"hands/{entry['device_model']}/"), asset_id
        else:
            raise AssertionError(
                f"unexpected device_kind for tracked robot asset: {asset_id}"
            )

        # Backend-specific runtime assets remain nested below the device.
        assert f"/{entry['backend']}/" in f"/{path}", asset_id

    text = registry_path.read_text(encoding="utf-8").lower()
    assert "phase2b0" not in text
    assert "/home/lyf" not in text
    assert "dexterous_robot_framework" not in text


def test_asset_root_example_uses_shared_robot_assets_root() -> None:
    root = Path(__file__).resolve().parents[2]
    path = root / "configs/assets/robot_assets.example.yaml"
    assert (
        path.read_text(encoding="utf-8").strip()
        == "robot_assets_root: ${ROBOT_ASSETS_ROOT}"
    )


def test_legacy_direct_path_asset_sample_is_removed() -> None:
    root = Path(__file__).resolve().parents[2]
    assert not (root / "configs/assets/local_assets.example.yaml").exists()
