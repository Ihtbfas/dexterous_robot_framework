#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import tarfile
from datetime import datetime, timezone
from pathlib import Path


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--review-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--classification", required=True)
    parser.add_argument("--status", required=True, choices=("PASS", "BLOCKED"))
    args = parser.parse_args()

    review = args.review_dir.resolve()
    output = args.output.resolve()
    review.mkdir(parents=True, exist_ok=True)
    output.parent.mkdir(parents=True, exist_ok=True)

    summary = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "classification": args.classification,
        "status": args.status,
    }
    (review / "review_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    files = sorted(p for p in review.rglob("*") if p.is_file() and p.name != "SHA256SUMS")
    lines = [f"{sha256(path)}  {path.relative_to(review).as_posix()}" for path in files]
    (review / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")

    with tarfile.open(output, mode="w:xz") as tf:
        tf.add(review, arcname="review")

    print(json.dumps({"archive": str(output), "sha256": sha256(output), **summary}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
