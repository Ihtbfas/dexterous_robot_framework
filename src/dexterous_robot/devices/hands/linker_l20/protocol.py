from __future__ import annotations

import math

from .types import L20ActiveCommand16, L20ProtocolCommand20

REVERSED_SLOTS = frozenset({0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 19})
SLOT_SOURCES: tuple[str | None, ...] = (
    "thumb_root_flex",
    "index_root_flex",
    "middle_root_flex",
    "ring_root_flex",
    "little_root_flex",
    "thumb_yaw",
    "index_yaw",
    "middle_yaw",
    "ring_yaw",
    "little_yaw",
    "thumb_roll",
    None,
    None,
    None,
    None,
    "thumb_tip_flex",
    "index_tip_flex",
    "middle_tip_flex",
    "ring_tip_flex",
    "little_tip_flex",
)
RESERVED_SLOTS = frozenset({11, 12, 13, 14})


def adapt_active_to_protocol20(command: L20ActiveCommand16) -> L20ProtocolCommand20:
    """Map Active16 to normalized protocol slot order, including direction reversal."""
    if not isinstance(command, L20ActiveCommand16):
        raise ValueError("L20_PROTOCOL_COMMAND_INVALID")
    active = command.as_mapping()
    values: list[float] = []
    for slot, source in enumerate(SLOT_SOURCES):
        if source is None:
            values.append(0.0)
        else:
            value = active[source]
            values.append((1.0 - value) if slot in REVERSED_SLOTS else value)
    return L20ProtocolCommand20(tuple(values))


def encode_official20(command: L20ActiveCommand16) -> tuple[int, ...]:
    """Encode one Active16 command into the frozen 20-byte official transport layout."""
    protocol = adapt_active_to_protocol20(command)
    return tuple(math.floor(255.0 * value) for value in protocol.values)
