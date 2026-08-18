from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _write_poison_pytest_plugin(root: Path) -> None:
    (root / "drf_poison_pytest_plugin.py").write_text(
        "raise RuntimeError('DRF_EXTERNAL_PYTEST_PLUGIN_WAS_AUTOLOADED')\n",
        encoding="utf-8",
    )
    dist = root / "drf_poison_pytest_plugin-1.0.dist-info"
    dist.mkdir()
    (dist / "METADATA").write_text(
        "Metadata-Version: 2.1\nName: drf-poison-pytest-plugin\nVersion: 1.0\n",
        encoding="utf-8",
    )
    (dist / "entry_points.txt").write_text(
        "[pytest11]\ndrf_poison = drf_poison_pytest_plugin\n",
        encoding="utf-8",
    )


def test_project_test_runner_disables_external_pytest_plugin_autoload(tmp_path):
    _write_poison_pytest_plugin(tmp_path)
    env = os.environ.copy()
    env.pop("PYTEST_DISABLE_PLUGIN_AUTOLOAD", None)
    existing = env.get("PYTHONPATH")
    env["PYTHONPATH"] = str(tmp_path) if not existing else f"{tmp_path}{os.pathsep}{existing}"

    proc = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "run_tests.py"), "tests/contract/test_repository_boundary.py", "-q"],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout
    assert "DRF_EXTERNAL_PYTEST_PLUGIN_WAS_AUTOLOADED" not in proc.stdout
