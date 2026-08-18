from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def test_importing_isaac_backend_package_does_not_import_omni_or_isaacsim():
    code = r'''
import sys
import dexterous_robot.backends.isaac
for prefix in ("omni", "isaacsim", "pxr", "warp", "usdrt"):
    assert prefix not in sys.modules, (prefix, sorted(k for k in sys.modules if k.startswith(prefix))[:10])
print("LAZY_IMPORT_BOUNDARY_PASS")
'''
    env = dict(os.environ)
    root = Path(__file__).resolve().parents[2]
    env["PYTHONPATH"] = str(root / "src")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False, env=env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "LAZY_IMPORT_BOUNDARY_PASS" in result.stdout
