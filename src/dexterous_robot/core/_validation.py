from __future__ import annotations

import math
from collections.abc import Iterable


def tuple_of_floats(values: Iterable[float], *, error_prefix: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{error_prefix}_INVALID") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{error_prefix}_NONFINITE")
    return result


def tuple_of_names(values: Iterable[str]) -> tuple[str, ...]:
    try:
        result = tuple(str(value) for value in values)
    except TypeError as exc:
        raise ValueError("JOINT_NAMES_INVALID") from exc
    if any(not name for name in result):
        raise ValueError("JOINT_NAMES_INVALID")
    if len(result) != len(set(result)):
        raise ValueError("JOINT_NAMES_DUPLICATE")
    return result
