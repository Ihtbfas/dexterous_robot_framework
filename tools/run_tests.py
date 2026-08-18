#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from collections.abc import Sequence


def configure_test_environment() -> None:
    # Keep project tests isolated from unrelated pytest11 entry points injected
    # through ROS/other global environments. Explicit project plugins can still
    # be loaded later with -p when intentionally required.
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"


def main(argv: Sequence[str] | None = None) -> int:
    configure_test_environment()
    import pytest

    args = list(sys.argv[1:] if argv is None else argv)
    return int(pytest.main(args))


if __name__ == "__main__":
    raise SystemExit(main())
