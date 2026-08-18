from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_required_gitignore_boundaries_are_declared():
    text = (ROOT / ".gitignore").read_text(encoding="utf-8")
    for pattern in ("configs/local/", "experiments/", "runs/", "evidence/", "scratch/"):
        assert pattern in text


def test_production_tree_contains_no_legacy_revision_names():
    forbidden = ("r15", "phase2b", "p2b2")
    offenders = []
    src = ROOT / "src"
    if src.exists():
        for path in src.rglob("*"):
            if path.is_file() and any(token in path.name.lower() for token in forbidden):
                offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []
