from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_isaac_config_no_longer_defines_or_exports_task_schema() -> None:
    text = (ROOT / "src/dexterous_robot/backends/isaac/config.py").read_text(encoding="utf-8")
    init_text = (ROOT / "src/dexterous_robot/backends/isaac/__init__.py").read_text(encoding="utf-8")
    assert "class TabletopGraspLiftConfig" not in text
    assert "def load_tabletop_grasp_lift_config" not in text
    assert "TabletopGraspLiftConfig" not in init_text
    assert "load_tabletop_grasp_lift_config" not in init_text


def test_motion_package_has_no_backend_imports() -> None:
    offenders = []
    for path in (ROOT / "src/dexterous_robot/motion").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        if "dexterous_robot.backends" in source or "isaac" in source.lower() or "mujoco" in source.lower():
            offenders.append(path.name)
    assert offenders == []
