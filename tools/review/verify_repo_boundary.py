#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

FORBIDDEN_SOURCE_TOKENS = ("phase2b", "p2b2", "r15u")
FORBIDDEN_ABSOLUTE_PREFIX = "/home/lyf/isaacsim_projects/"
IGNORED_TOP_LEVEL = {"experiments", "runs", "evidence", "scratch"}


def scan(root: Path) -> dict[str, object]:
    src = root / "src"
    source_name_offenders: list[str] = []
    absolute_path_offenders: list[str] = []
    if src.exists():
        for path in src.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            lowered = path.name.lower()
            if any(token in lowered for token in FORBIDDEN_SOURCE_TOKENS):
                source_name_offenders.append(rel)
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            if FORBIDDEN_ABSOLUTE_PREFIX in text:
                absolute_path_offenders.append(rel)

    gitignore = (root / ".gitignore").read_text(encoding="utf-8") if (root / ".gitignore").exists() else ""
    missing_ignores = [f"{name}/" for name in sorted(IGNORED_TOP_LEVEL) if f"{name}/" not in gitignore]
    if "configs/local/" not in gitignore:
        missing_ignores.append("configs/local/")

    return {
        "source_name_offenders": source_name_offenders,
        "absolute_path_offenders": absolute_path_offenders,
        "missing_ignores": missing_ignores,
        "pass": not (source_name_offenders or absolute_path_offenders or missing_ignores),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    result = scan(args.root.resolve())
    text = json.dumps(result, indent=2, sort_keys=True)
    print(text)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(text + "\n", encoding="utf-8")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
