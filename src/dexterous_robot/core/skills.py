from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SkillStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class FailureReason(Enum):
    NONE = "NONE"
    OBJECT_SLIPPED = "OBJECT_SLIPPED"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    GRASP_NOT_ESTABLISHED = "GRASP_NOT_ESTABLISHED"
    TIMEOUT = "TIMEOUT"
    RUNTIME_ERROR = "RUNTIME_ERROR"


@dataclass(frozen=True)
class SkillResult:
    status: SkillStatus
    reason: FailureReason = FailureReason.NONE
    message: str = ""
