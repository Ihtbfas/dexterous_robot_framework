from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class LocalAssetConfig:
    wam_runtime: Path
    l20_runtime: Path
