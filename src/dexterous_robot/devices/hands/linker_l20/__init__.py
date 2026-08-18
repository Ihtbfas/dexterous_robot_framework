"""Linker Hand L20 backend-independent command, mapping, and protocol model."""

from .mapping import COUPLING_MULTIPLIERS, L20_JOINT_SPECS, map_active_to_physical
from .model import LinkerL20Model
from .protocol import REVERSED_SLOTS, SLOT_SOURCES, adapt_active_to_protocol20, encode_official20
from .types import (
    L20_ACTIVE_CHANNELS,
    L20_PHYSICAL_JOINTS,
    L20ActiveCommand16,
    L20PhysicalState21,
    L20PhysicalTarget21,
    L20ProtocolCommand20,
)

__all__ = [
    "COUPLING_MULTIPLIERS",
    "L20_ACTIVE_CHANNELS",
    "L20_JOINT_SPECS",
    "L20_PHYSICAL_JOINTS",
    "L20ActiveCommand16",
    "L20PhysicalState21",
    "L20PhysicalTarget21",
    "L20ProtocolCommand20",
    "LinkerL20Model",
    "REVERSED_SLOTS",
    "SLOT_SOURCES",
    "adapt_active_to_protocol20",
    "encode_official20",
    "map_active_to_physical",
]
