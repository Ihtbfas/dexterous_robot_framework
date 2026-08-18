from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Sequence


def _path(value: str, error: str) -> str:
    if not isinstance(value, str) or not value.startswith("/"):
        raise ValueError(error)
    return value


def _normal(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError("ISAAC_CONTACT_NORMAL_INVALID")
    result = tuple(float(value) for value in values)
    if not all(isfinite(value) for value in result):
        raise ValueError("ISAAC_CONTACT_NORMAL_INVALID")
    return result  # type: ignore[return-value]


@dataclass(frozen=True)
class ContactSample:
    """One object-centric contact impulse sample from the Isaac adapter.

    `normal_xyz` is normalized by the collector so it is expressed from the
    tracked object toward the other body. Core/task code never sees raw PhysX
    contact headers.
    """

    object_path: str
    other_path: str
    normal_xyz: tuple[float, float, float]
    impulse_magnitude_ns: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "object_path", _path(self.object_path, "ISAAC_CONTACT_OBJECT_PATH_INVALID"))
        object.__setattr__(self, "other_path", _path(self.other_path, "ISAAC_CONTACT_OTHER_PATH_INVALID"))
        object.__setattr__(self, "normal_xyz", _normal(self.normal_xyz))
        impulse = float(self.impulse_magnitude_ns)
        if not isfinite(impulse) or impulse < 0.0:
            raise ValueError("ISAAC_CONTACT_IMPULSE_INVALID")
        object.__setattr__(self, "impulse_magnitude_ns", impulse)


@dataclass(frozen=True)
class ContactSummary:
    object_table_normal_n: float
    opposing_y_squeeze_n: float


def summarize_contacts(
    samples: Iterable[ContactSample],
    *,
    dt_s: float,
    object_path: str,
    table_path: str,
) -> ContactSummary:
    dt = float(dt_s)
    if not isfinite(dt) or dt <= 0.0:
        raise ValueError("ISAAC_CONTACT_DT_INVALID")
    object_path = _path(object_path, "ISAAC_CONTACT_OBJECT_PATH_INVALID")
    table_path = _path(table_path, "ISAAC_CONTACT_TABLE_PATH_INVALID")

    table_normal = 0.0
    positive_y = 0.0
    negative_y = 0.0
    for sample in samples:
        if sample.object_path != object_path:
            continue
        force_n = sample.impulse_magnitude_ns / dt
        nx, ny, nz = sample.normal_xyz
        if sample.other_path == table_path:
            table_normal += max(0.0, nz) * force_n
        if ny > 0.0:
            positive_y += ny * force_n
        elif ny < 0.0:
            negative_y += (-ny) * force_n
    return ContactSummary(
        object_table_normal_n=float(table_normal),
        opposing_y_squeeze_n=float(min(positive_y, negative_y)),
    )


class IsaacContactCollector:
    """Runtime-only adapter from raw PhysX contact reports to ContactSample."""

    def __init__(self, *, physics_schema_tools, simulation_interface, object_path: str) -> None:
        self._tools = physics_schema_tools
        self._interface = simulation_interface
        self._object_path = _path(object_path, "ISAAC_CONTACT_OBJECT_PATH_INVALID")
        self._samples: list[ContactSample] = []
        self._subscription = None

    def subscribe(self) -> None:
        if self._subscription is not None:
            return
        self._subscription = self._interface.subscribe_physics_contact_report_events(self._callback)

    def clear(self) -> None:
        self._samples.clear()

    def snapshot(self) -> tuple[ContactSample, ...]:
        return tuple(self._samples)

    def _as_path(self, value) -> str:
        return str(self._tools.intToSdfPath(value))

    @staticmethod
    def _vec(value) -> tuple[float, ...]:
        try:
            return tuple(float(x) for x in value)
        except TypeError:
            return (float(value),)

    def _callback(self, headers, data, friction) -> None:  # pragma: no cover - requires Isaac runtime
        del friction
        for header in headers:
            body0 = self._as_path(header.actor0)
            body1 = self._as_path(header.actor1)
            collider0 = self._as_path(header.collider0)
            collider1 = self._as_path(header.collider1)
            side0 = self._object_path in (body0, collider0)
            side1 = self._object_path in (body1, collider1)
            if not side0 and not side1:
                continue
            if side0:
                other = body1 if body1 != self._object_path else collider1
                direction = 1.0
            else:
                other = body0 if body0 != self._object_path else collider0
                direction = -1.0
            offset = int(header.contact_data_offset)
            count = int(header.num_contact_data)
            for index in range(offset, offset + count):
                item = data[index]
                normal_raw = self._vec(item.normal)
                if len(normal_raw) != 3:
                    continue
                normal = tuple(direction * value for value in normal_raw)
                impulse_raw = self._vec(item.impulse)
                if len(impulse_raw) == 1:
                    impulse = abs(impulse_raw[0])
                else:
                    impulse = sum(value * value for value in impulse_raw) ** 0.5
                self._samples.append(
                    ContactSample(
                        object_path=self._object_path,
                        other_path=other,
                        normal_xyz=normal,  # type: ignore[arg-type]
                        impulse_magnitude_ns=impulse,
                    )
                )
