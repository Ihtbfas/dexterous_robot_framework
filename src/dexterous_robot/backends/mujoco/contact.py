from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Literal, Sequence


ContactKind = Literal["object_table", "robot_object"]
_SIDE_EPSILON_M = 1.0e-9


@dataclass(frozen=True)
class TabletopContactSample:
    kind: ContactKind
    normal_force_n: float
    position_world_m: tuple[float, float, float]
    normal_world: tuple[float, float, float]

    def __post_init__(self) -> None:
        if self.kind not in ("object_table", "robot_object"):
            raise ValueError("MUJOCO_TABLETOP_CONTACT_KIND_INVALID")
        force = float(self.normal_force_n)
        if not math.isfinite(force) or force < 0.0:
            raise ValueError("MUJOCO_TABLETOP_CONTACT_FORCE_INVALID")

        position = tuple(float(v) for v in self.position_world_m)
        normal = tuple(float(v) for v in self.normal_world)
        if (
            len(position) != 3
            or len(normal) != 3
            or not all(math.isfinite(v) for v in (*position, *normal))
        ):
            raise ValueError("MUJOCO_TABLETOP_CONTACT_VECTOR_INVALID")

        object.__setattr__(self, "normal_force_n", force)
        object.__setattr__(self, "position_world_m", position)
        object.__setattr__(self, "normal_world", normal)


@dataclass(frozen=True)
class TabletopContactTelemetry:
    opposing_y_squeeze_n: float
    object_table_normal_n: float


def reduce_tabletop_contacts(
    samples: Iterable[TabletopContactSample],
    *,
    object_center_world_m: Sequence[float],
) -> TabletopContactTelemetry:
    center = tuple(float(v) for v in object_center_world_m)
    if (
        len(center) != 3
        or not all(math.isfinite(v) for v in center)
    ):
        raise ValueError("MUJOCO_TABLETOP_OBJECT_CENTER_INVALID")

    positive_y_n = 0.0
    negative_y_n = 0.0
    table_normal_n = 0.0

    for sample in tuple(samples):
        if not isinstance(sample, TabletopContactSample):
            raise TypeError("MUJOCO_TABLETOP_CONTACT_SAMPLE_INVALID")

        if sample.kind == "object_table":
            table_normal_n += sample.normal_force_n
            continue

        y_offset = sample.position_world_m[1] - center[1]
        if abs(y_offset) <= _SIDE_EPSILON_M:
            continue

        projected_y_n = (
            sample.normal_force_n * abs(sample.normal_world[1])
        )
        if y_offset > 0.0:
            positive_y_n += projected_y_n
        else:
            negative_y_n += projected_y_n

    return TabletopContactTelemetry(
        opposing_y_squeeze_n=min(positive_y_n, negative_y_n),
        object_table_normal_n=table_normal_n,
    )
