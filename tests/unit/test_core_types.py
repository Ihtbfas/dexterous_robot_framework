import dataclasses
import math

import pytest

from dexterous_robot.core.commands import JointEffortCommand, JointPositionCommand
from dexterous_robot.core.geometry import Pose
from dexterous_robot.core.joints import JointState
from dexterous_robot.core.skills import FailureReason, SkillResult, SkillStatus


def test_pose_is_frozen_and_exact_width():
    pose = Pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), "world")
    with pytest.raises(dataclasses.FrozenInstanceError):
        pose.frame_id = "other"
    with pytest.raises(ValueError, match="POSE_POSITION_INVALID"):
        Pose((1.0, 2.0), (0.0, 0.0, 0.0, 1.0), "world")
    with pytest.raises(ValueError, match="POSE_QUATERNION_INVALID"):
        Pose((1.0, 2.0, 3.0), (0.0, 0.0, 1.0), "world")


def test_pose_rejects_non_finite_values():
    with pytest.raises(ValueError, match="POSE_POSITION_NONFINITE"):
        Pose((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "world")
    with pytest.raises(ValueError, match="POSE_QUATERNION_NONFINITE"):
        Pose((0.0, 0.0, 0.0), (0.0, 0.0, math.inf, 1.0), "world")


def test_joint_state_rejects_width_mismatch():
    with pytest.raises(ValueError, match="JOINT_STATE_WIDTH_MISMATCH"):
        JointState(("j1", "j2"), (0.0,), (0.0, 0.0), None)
    with pytest.raises(ValueError, match="JOINT_STATE_WIDTH_MISMATCH"):
        JointState(("j1", "j2"), (0.0, 0.0), (0.0, 0.0), (0.0,))


def test_joint_state_rejects_duplicate_names_and_non_finite_values():
    with pytest.raises(ValueError, match="JOINT_NAMES_DUPLICATE"):
        JointState(("j1", "j1"), (0.0, 0.0), (0.0, 0.0), None)
    with pytest.raises(ValueError, match="JOINT_STATE_POSITION_NONFINITE"):
        JointState(("j1",), (math.inf,), (0.0,), None)


def test_joint_state_normalizes_sequence_inputs_to_immutable_tuples():
    state = JointState(["j1"], [0.1], [0.2], [0.3])
    assert state.names == ("j1",)
    assert state.position_rad == (0.1,)
    assert state.velocity_rad_s == (0.2,)
    assert state.effort_nm == (0.3,)
    with pytest.raises(dataclasses.FrozenInstanceError):
        state.names = ("j2",)


def test_joint_position_command_validates_width_names_and_values():
    command = JointPositionCommand("hand", ["j1", "j2"], [0.1, 0.2], "hold")
    assert command.joint_names == ("j1", "j2")
    assert command.position_rad == (0.1, 0.2)
    with pytest.raises(ValueError, match="JOINT_POSITION_COMMAND_WIDTH_MISMATCH"):
        JointPositionCommand("hand", ("j1", "j2"), (0.1,), None)
    with pytest.raises(ValueError, match="JOINT_NAMES_DUPLICATE"):
        JointPositionCommand("hand", ("j1", "j1"), (0.1, 0.2), None)
    with pytest.raises(ValueError, match="JOINT_POSITION_COMMAND_NONFINITE"):
        JointPositionCommand("hand", ("j1",), (math.nan,), None)


def test_joint_effort_command_validates_width_and_values():
    command = JointEffortCommand("arm", ["j1"], [1.2])
    assert command.joint_names == ("j1",)
    assert command.effort_nm == (1.2,)
    with pytest.raises(ValueError, match="JOINT_EFFORT_COMMAND_WIDTH_MISMATCH"):
        JointEffortCommand("arm", ("j1", "j2"), (1.0,))
    with pytest.raises(ValueError, match="JOINT_EFFORT_COMMAND_NONFINITE"):
        JointEffortCommand("arm", ("j1",), (math.inf,))


def test_skill_result_is_semantic_only():
    result = SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "lost object")
    assert result.status is SkillStatus.FAILURE
    assert result.reason is FailureReason.OBJECT_SLIPPED
    assert result.message == "lost object"
