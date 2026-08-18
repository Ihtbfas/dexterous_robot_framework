#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from dexterous_robot.golden import BLOCK_CLASSIFICATION, evaluate_m1_golden


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        summary = json.loads(args.summary.read_text(encoding="utf-8"))
    except Exception as exc:
        result = {"status": "BLOCKED", "classification": BLOCK_CLASSIFICATION, "errors": [f"SUMMARY_READ_FAILED:{type(exc).__name__}:{exc}"], "gates": {}}
    else:
        result = evaluate_m1_golden(summary)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
