from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
from typing import Any, Sequence

from dexterous_robot.backends.base import Backend, BackendState
from dexterous_robot.config.tasks import TabletopGraspLiftConfig
from dexterous_robot.core import (
    Command,
    JointEffortCommand,
    JointPositionCommand,
    JointState,
    Pose,
    RigidBodyKinematicCommand,
)
from dexterous_robot.robots import ManipulatorSystem

from .config import MuJoCoBackendConfig
from .contact import TabletopContactSample, reduce_tabletop_contacts
from .model import assemble_wam7_l20_model
from .scene import (
    MuJoCoTabletopSceneHandles,
    MuJoCoTabletopSceneIds,
    author_tabletop_scene,
    resolve_tabletop_scene_ids,
)
from .timing import resolve_substeps
from .topology import MuJoCoRouting, build_mujoco_routing


_WAM_B1_POSITION_KP = 80.0
_WAM_B1_POSITION_KV = 8.0
_WAM_B2_ACTUATOR_KV = 0.0
_WAM_B2_PASSIVE_DAMPING = 9.0
_ARM_ACCEPTED_PROFILES = {None, "arm_carry_position_drive"}
_HAND_ACCEPTED_PROFILES = {None, "hand_open_hold", "hand_grasp_lock"}

_B1_ROBOT_COLLISION_CONTYPE = 1
_B1_ROBOT_COLLISION_CONAFFINITY = 2


def _collision_masks_compatible(
    contype_a: int,
    conaffinity_a: int,
    contype_b: int,
    conaffinity_b: int,
) -> bool:
    return bool(
        (int(contype_a) & int(conaffinity_b))
        or (int(contype_b) & int(conaffinity_a))
    )


def _apply_b1_robot_internal_collision_policy(spec: Any) -> dict[str, object]:
    """Normalize robot-only B1 collision geoms before compilation.

    The B1 composite contains robot assets only. Existing visual-only geoms
    (0/0) remain non-colliding. Existing collision geoms are moved to a
    robot-only contact class that is incompatible with itself but compatible
    with a future default external geom (1/1).
    """

    explicit_pairs = tuple(spec.pairs)
    if explicit_pairs:
        raise RuntimeError(
            "MUJOCO_B1_EXPLICIT_CONTACT_PAIRS_UNSUPPORTED:"
            f"{len(explicit_pairs)}"
        )

    normalized = 0
    visual_only = 0
    for geom in spec.geoms:
        contype = int(geom.contype)
        conaffinity = int(geom.conaffinity)
        if contype == 0 and conaffinity == 0:
            visual_only += 1
            continue

        geom.contype = _B1_ROBOT_COLLISION_CONTYPE
        geom.conaffinity = _B1_ROBOT_COLLISION_CONAFFINITY
        normalized += 1

    if normalized == 0:
        raise RuntimeError("MUJOCO_B1_NO_COLLISION_GEOMS_FOUND")

    robot_self_compatible = _collision_masks_compatible(
        _B1_ROBOT_COLLISION_CONTYPE,
        _B1_ROBOT_COLLISION_CONAFFINITY,
        _B1_ROBOT_COLLISION_CONTYPE,
        _B1_ROBOT_COLLISION_CONAFFINITY,
    )
    external_default_compatible = _collision_masks_compatible(
        _B1_ROBOT_COLLISION_CONTYPE,
        _B1_ROBOT_COLLISION_CONAFFINITY,
        1,
        1,
    )
    if robot_self_compatible or not external_default_compatible:
        raise RuntimeError("MUJOCO_B1_COLLISION_MASK_POLICY_INVALID")

    return {
        "policy": "robot_internal_filtered_external_default_compatible",
        "robot_contype": _B1_ROBOT_COLLISION_CONTYPE,
        "robot_conaffinity": _B1_ROBOT_COLLISION_CONAFFINITY,
        "normalized_collision_geom_count": normalized,
        "preserved_visual_only_geom_count": visual_only,
        "explicit_pair_count": 0,
        "robot_self_compatible": robot_self_compatible,
        "external_default_compatible": external_default_compatible,
    }


class MuJoCoBackend(Backend):
    """MuJoCo backend with lazy simulator import and canonical typed routing."""

    def __init__(
        self,
        *,
        robot: ManipulatorSystem,
        backend_config: MuJoCoBackendConfig,
        arm_runtime: Path,
        hand_runtime: Path,
    ) -> None:
        if not isinstance(robot, ManipulatorSystem):
            raise TypeError("MUJOCO_BACKEND_ROBOT_INVALID")
        if not isinstance(backend_config, MuJoCoBackendConfig):
            raise TypeError("MUJOCO_BACKEND_CONFIG_INVALID")
        if not isinstance(arm_runtime, Path):
            raise TypeError("MUJOCO_ARM_RUNTIME_PATH_INVALID")
        if not isinstance(hand_runtime, Path):
            raise TypeError("MUJOCO_HAND_RUNTIME_PATH_INVALID")

        self._robot = robot
        self._cfg = backend_config
        self._arm_runtime = arm_runtime.expanduser()
        self._hand_runtime = hand_runtime.expanduser()

        self._initialized = False
        self._mujoco: Any | None = None
        self._spec: Any | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._routing: MuJoCoRouting | None = None
        self._initial_qpos: tuple[float, ...] | None = None
        self._wam_flange_body_name: str | None = None
        self._l20_base_body_name: str = self._cfg.frames.l20_base
        self._tabletop_task_config: TabletopGraspLiftConfig | None = None
        self._tabletop_scene_handles: MuJoCoTabletopSceneHandles | None = None
        self._tabletop_scene_ids: MuJoCoTabletopSceneIds | None = None
        self._diagnostics: dict[str, object] = {}

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def diagnostics(self) -> dict[str, object]:
        return dict(self._diagnostics)

    def configure_tabletop_scene(
        self,
        task_config: TabletopGraspLiftConfig,
    ) -> None:
        if self._initialized:
            raise RuntimeError(
                "MUJOCO_TABLETOP_SCENE_CONFIGURE_AFTER_INITIALIZE"
            )
        if self._tabletop_task_config is not None:
            raise RuntimeError(
                "MUJOCO_TABLETOP_SCENE_ALREADY_CONFIGURED"
            )
        if not isinstance(task_config, TabletopGraspLiftConfig):
            raise TypeError("MUJOCO_TABLETOP_TASK_CONFIG_INVALID")
        self._tabletop_task_config = task_config

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _verify_runtime_assets(self) -> dict[str, str]:
        rows = {
            "arm_runtime": self._arm_runtime,
            "hand_runtime": self._hand_runtime,
        }
        hashes: dict[str, str] = {}
        for label, path in rows.items():
            if not path.is_file():
                raise RuntimeError(f"MUJOCO_ASSET_MISSING:{label}:{path}")
            hashes[label] = self._sha256(path)
        return hashes

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("MUJOCO_BACKEND_NOT_INITIALIZED")

    def _clear_runtime_handles(self) -> None:
        self._initialized = False
        self._mujoco = None
        self._spec = None
        self._model = None
        self._data = None
        self._routing = None
        self._initial_qpos = None
        self._wam_flange_body_name = None
        self._tabletop_scene_handles = None
        self._tabletop_scene_ids = None

    @staticmethod
    def _augment_wam_position_actuation(
        mujoco: Any,
        spec: Any,
        joint_names: tuple[str, ...],
    ) -> tuple[str, ...]:
        existing_targets = {
            str(actuator.target)
            for actuator in spec.actuators
            if int(actuator.trntype) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and str(actuator.target)
        }
        unexpected = sorted(set(joint_names) & existing_targets)
        if unexpected:
            raise RuntimeError(
                "MUJOCO_WAM_SOURCE_NOT_PASSIVE:" + ",".join(unexpected)
            )

        names: list[str] = []
        for joint_name in joint_names:
            actuator_name = f"wam_b1_position__{joint_name}"
            if spec.actuator(actuator_name) is not None:
                raise RuntimeError(
                    f"MUJOCO_WAM_ACTUATOR_NAME_COLLISION:{actuator_name}"
                )

            actuator = spec.add_actuator(
                name=actuator_name,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                target=joint_name,
            )
            actuator.gear[0] = 1.0
            actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            actuator.gainprm[0] = _WAM_B1_POSITION_KP
            actuator.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            actuator.biasprm[1] = -_WAM_B1_POSITION_KP
            actuator.biasprm[2] = -_WAM_B2_ACTUATOR_KV
            names.append(actuator_name)
        return tuple(names)

    @staticmethod
    def _apply_wam_b2_task_servo_policy(
        model: Any,
        routing: MuJoCoRouting,
        joint_names: tuple[str, ...],
    ) -> None:
        for joint_name in joint_names:
            joint = routing.joint_by_name[joint_name]
            actuator = routing.actuator_by_joint[joint_name]
            model.dof_damping[joint.qvel_adr] = _WAM_B2_PASSIVE_DAMPING
            model.actuator_biasprm[
                actuator.actuator_id, 2
            ] = -_WAM_B2_ACTUATOR_KV

    def initialize(self) -> None:
        if self._initialized:
            raise RuntimeError("MUJOCO_BACKEND_ALREADY_INITIALIZED")

        self._diagnostics = {"phase": "asset_verification"}
        try:
            asset_hashes = self._verify_runtime_assets()
            self._diagnostics["asset_hashes"] = dict(asset_hashes)

            self._diagnostics["phase"] = "model_loading"
            import mujoco

            composite = assemble_wam7_l20_model(
                arm_runtime=self._arm_runtime,
                hand_runtime=self._hand_runtime,
                mount_position_xyz_m=tuple(
                    self._robot.hand_mount.pose.position_xyz_m
                ),
                mount_quaternion_xyzw=tuple(
                    self._robot.hand_mount.pose.quaternion_xyzw
                ),
                logical_wam_flange=self._cfg.frames.wam_flange,
                logical_l20_base=self._cfg.frames.l20_base,
                expected_physics_timestep_s=self._cfg.physics_timestep_s,
            )

            self._mujoco = mujoco
            self._spec = composite.spec
            self._wam_flange_body_name = str(
                composite.metadata["resolved_wam_flange_body"]
            )

            collision_policy = _apply_b1_robot_internal_collision_policy(
                self._spec
            )
            self._diagnostics["robot_internal_collision_policy"] = (
                collision_policy
            )

            added_wam_actuators = self._augment_wam_position_actuation(
                mujoco,
                self._spec,
                tuple(self._robot.arm.joint_names),
            )

            if self._tabletop_task_config is not None:
                self._tabletop_scene_handles = author_tabletop_scene(
                    mujoco=mujoco,
                    spec=self._spec,
                    task_config=self._tabletop_task_config,
                )

            self._model = self._spec.compile()
            self._data = mujoco.MjData(self._model)
            mujoco.mj_forward(self._model, self._data)

            self._diagnostics["phase"] = "model_compiled"
            self._diagnostics["model"] = {
                "nq": int(self._model.nq),
                "nv": int(self._model.nv),
                "nu": int(self._model.nu),
                "njnt": int(self._model.njnt),
                "nbody": int(self._model.nbody),
                "ngeom": int(self._model.ngeom),
                "physics_timestep_s": float(self._model.opt.timestep),
            }
            self._diagnostics["wam_position_actuation"] = {
                "policy": "backend_augmented_b1_position_servo",
                "kp": _WAM_B1_POSITION_KP,
                "kv": _WAM_B2_ACTUATOR_KV,
                "legacy_b1_kv_reference": _WAM_B1_POSITION_KV,
                "task_servo_policy": "wam_passive_dof_damping_v1",
                "passive_dof_damping": _WAM_B2_PASSIVE_DAMPING,
                "bias_feedforward": "qfrc_bias_at_runtime_step_start",
                "actuator_names": list(added_wam_actuators),
            }

            self._diagnostics["phase"] = "topology_resolution"
            self._routing = build_mujoco_routing(self._model)
            if len(self._routing.arm_actuators) != len(
                self._robot.arm.joint_names
            ):
                raise RuntimeError(
                    "MUJOCO_WAM_ACTUATOR_ROUTING_INCOMPLETE:"
                    f"{len(self._routing.arm_actuators)}"
                )
            self._apply_wam_b2_task_servo_policy(
                self._model,
                self._routing,
                tuple(self._robot.arm.joint_names),
            )

            if self._tabletop_scene_handles is not None:
                self._tabletop_scene_ids = resolve_tabletop_scene_ids(
                    mujoco=mujoco,
                    model=self._model,
                    handles=self._tabletop_scene_handles,
                )
                self._diagnostics["tabletop_scene"] = {
                    "enabled": True,
                    "table_geom_name": self._tabletop_scene_handles.table_geom_name,
                    "object_body_name": self._tabletop_scene_handles.object_body_name,
                    "object_geom_name": self._tabletop_scene_handles.object_geom_name,
                    "object_free_joint_name": self._tabletop_scene_handles.object_free_joint_name,
                    "object_anchor_body_name": self._tabletop_scene_handles.object_anchor_body_name,
                    "object_weld_name": self._tabletop_scene_handles.object_weld_name,
                    "table_geom_id": self._tabletop_scene_ids.table_geom_id,
                    "object_body_id": self._tabletop_scene_ids.object_body_id,
                    "object_geom_id": self._tabletop_scene_ids.object_geom_id,
                    "object_free_joint_id": self._tabletop_scene_ids.object_free_joint_id,
                    "object_anchor_body_id": self._tabletop_scene_ids.object_anchor_body_id,
                    "object_weld_id": self._tabletop_scene_ids.object_weld_id,
                }
            else:
                self._diagnostics["tabletop_scene"] = {"enabled": False}

            self._initial_qpos = tuple(
                float(value) for value in self._model.qpos0
            )

            self._reset_data_without_lifecycle_check()

            self._diagnostics["phase"] = "initial_state_sampling"
            self._sample_state()

            self._initialized = True
            self._diagnostics["phase"] = "ready"
        except Exception as exc:
            self._initialized = False
            self._diagnostics["failure"] = f"{type(exc).__name__}:{exc}"
            self._clear_runtime_handles()
            raise

    def _reset_data_without_lifecycle_check(self) -> None:
        if (
            self._mujoco is None
            or self._model is None
            or self._data is None
            or self._initial_qpos is None
        ):
            raise RuntimeError("MUJOCO_BACKEND_RUNTIME_HANDLES_UNAVAILABLE")

        self._mujoco.mj_resetData(self._model, self._data)
        self._data.qpos[:] = self._initial_qpos
        self._data.qvel[:] = 0.0
        if int(self._model.nu) > 0:
            self._data.ctrl[:] = 0.0
        self._data.time = 0.0
        self._mujoco.mj_forward(self._model, self._data)

    @staticmethod
    def _joint_state(
        addresses: tuple[Any, ...],
        data: Any,
    ) -> JointState:
        return JointState(
            names=tuple(item.joint_name for item in addresses),
            position_rad=tuple(
                float(data.qpos[item.qpos_adr]) for item in addresses
            ),
            velocity_rad_s=tuple(
                float(data.qvel[item.qvel_adr]) for item in addresses
            ),
            effort_nm=None,
        )

    def _body_pose(self, body_name: str) -> Pose:
        if self._mujoco is None or self._model is None or self._data is None:
            raise RuntimeError("MUJOCO_BACKEND_RUNTIME_HANDLES_UNAVAILABLE")

        body_id = int(
            self._mujoco.mj_name2id(
                self._model,
                self._mujoco.mjtObj.mjOBJ_BODY,
                body_name,
            )
        )
        if body_id < 0:
            raise RuntimeError(f"MUJOCO_BODY_POSE_UNRESOLVED:{body_name}")

        position = tuple(float(value) for value in self._data.xpos[body_id])
        w, x, y, z = (
            float(value) for value in self._data.xquat[body_id]
        )
        return Pose(
            position_xyz_m=position,
            quaternion_xyzw=(x, y, z, w),
            frame_id="world",
        )


    def _tabletop_contact_signals(self) -> dict[str, float]:
        if (
            self._mujoco is None
            or self._model is None
            or self._data is None
            or self._tabletop_scene_ids is None
        ):
            raise RuntimeError(
                "MUJOCO_TABLETOP_CONTACT_HANDLES_UNAVAILABLE"
            )

        ids = self._tabletop_scene_ids
        samples: list[TabletopContactSample] = []
        object_center = tuple(
            float(value)
            for value in self._data.xpos[ids.object_body_id]
        )

        for index in range(int(self._data.ncon)):
            contact = self._data.contact[index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            pair = {geom1, geom2}

            if pair == {ids.object_geom_id, ids.table_geom_id}:
                kind = "object_table"
            elif (
                ids.object_geom_id in pair
                and ids.table_geom_id not in pair
            ):
                kind = "robot_object"
            else:
                continue

            force = np.zeros(6, dtype=float)
            self._mujoco.mj_contactForce(
                self._model,
                self._data,
                index,
                force,
            )
            frame = np.asarray(
                contact.frame,
                dtype=float,
            ).reshape(-1)
            position = np.asarray(
                contact.pos,
                dtype=float,
            ).reshape(-1)

            samples.append(
                TabletopContactSample(
                    kind=kind,
                    normal_force_n=max(0.0, float(force[0])),
                    position_world_m=(
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ),
                    normal_world=(
                        float(frame[0]),
                        float(frame[1]),
                        float(frame[2]),
                    ),
                )
            )

        telemetry = reduce_tabletop_contacts(
            samples,
            object_center_world_m=object_center,
        )
        return {
            "opposing_y_squeeze_n": telemetry.opposing_y_squeeze_n,
            "object_table_normal_n": telemetry.object_table_normal_n,
        }

    def _sample_state(self) -> BackendState:
        if (
            self._routing is None
            or self._data is None
            or self._wam_flange_body_name is None
        ):
            raise RuntimeError("MUJOCO_BACKEND_STATE_HANDLES_UNAVAILABLE")

        wam_j7_pose = self._body_pose(self._wam_flange_body_name)
        l20_base_pose = self._body_pose(self._l20_base_body_name)
        body_poses = {
            "wam_j7": wam_j7_pose,
            "l20_base": l20_base_pose,
        }
        if self._tabletop_scene_handles is not None:
            body_poses["hand_tcp"] = l20_base_pose
            body_poses["object"] = self._body_pose(
                self._tabletop_scene_handles.object_body_name
            )

        signals: dict[str, float | str] = {
            "backend": "mujoco",
            "sim_time_s": float(self._data.time),
        }
        if self._tabletop_scene_ids is not None:
            signals.update(self._tabletop_contact_signals())

        return BackendState(
            device_states={
                self._robot.arm.device_id: self._joint_state(
                    self._routing.arm_joints,
                    self._data,
                ),
                self._robot.hand.device_id: self._joint_state(
                    self._routing.hand_joints,
                    self._data,
                ),
            },
            body_poses=body_poses,
            signals=signals,
        )

    def read_state(self) -> BackendState:
        self._require_initialized()
        return self._sample_state()

    @staticmethod
    def _validate_command_joint_set(
        command: JointPositionCommand,
        expected_names: tuple[str, ...],
        *,
        error_prefix: str,
    ) -> None:
        names = command.joint_names
        if len(set(names)) != len(names):
            raise ValueError("MUJOCO_COMMAND_JOINT_DUPLICATE")
        if len(names) != len(expected_names) or set(names) != set(
            expected_names
        ):
            raise ValueError(f"{error_prefix}_COMMAND_JOINT_SET_INVALID")

    def _stage_position_command(
        self,
        command: JointPositionCommand,
        staged: dict[int, float],
    ) -> None:
        if self._routing is None:
            raise RuntimeError("MUJOCO_BACKEND_ROUTING_UNAVAILABLE")

        if command.device_id == self._robot.arm.device_id:
            expected = tuple(self._robot.arm.joint_names)
            self._validate_command_joint_set(
                command,
                expected,
                error_prefix="MUJOCO_ARM",
            )
            if command.profile not in _ARM_ACCEPTED_PROFILES:
                raise ValueError(
                    f"MUJOCO_ARM_COMMAND_PROFILE_INVALID:{command.profile}"
                )
        elif command.device_id == self._robot.hand.device_id:
            expected = tuple(self._robot.hand.physical_joints)
            self._validate_command_joint_set(
                command,
                expected,
                error_prefix="MUJOCO_HAND",
            )
            if command.profile not in _HAND_ACCEPTED_PROFILES:
                raise ValueError(
                    f"MUJOCO_HAND_COMMAND_PROFILE_INVALID:{command.profile}"
                )
        else:
            raise ValueError(
                f"MUJOCO_COMMAND_DEVICE_UNKNOWN:{command.device_id}"
            )

        actuator_map = self._routing.actuator_by_joint
        for joint_name, target in zip(
            command.joint_names,
            command.position_rad,
            strict=True,
        ):
            address = actuator_map.get(joint_name)
            if address is None:
                raise RuntimeError(
                    f"MUJOCO_POSITION_ACTUATOR_MISSING:{joint_name}"
                )
            if address.ctrl_adr in staged:
                raise ValueError(
                    f"MUJOCO_COMMAND_TARGET_DUPLICATE:{joint_name}"
                )
            staged[address.ctrl_adr] = float(target)

    def _validate_rigid_body_kinematic_command(
        self,
        command: RigidBodyKinematicCommand,
    ) -> bool:
        if self._tabletop_scene_handles is None or self._tabletop_scene_ids is None:
            raise ValueError(
                "MUJOCO_RIGID_BODY_KINEMATIC_SCENE_UNAVAILABLE"
            )
        if command.body_id != "object":
            raise ValueError(
                "MUJOCO_RIGID_BODY_KINEMATIC_BODY_UNKNOWN:"
                f"{command.body_id}"
            )
        return bool(command.kinematic_enabled)

    def apply(self, commands: Sequence[Command]) -> None:
        self._require_initialized()
        if self._data is None:
            raise RuntimeError("MUJOCO_BACKEND_RUNTIME_HANDLES_UNAVAILABLE")

        staged: dict[int, float] = {}
        staged_object_kinematic: bool | None = None
        for command in tuple(commands):
            if isinstance(command, JointPositionCommand):
                self._stage_position_command(command, staged)
            elif isinstance(command, RigidBodyKinematicCommand):
                if (
                    self._tabletop_scene_handles is None
                    or self._tabletop_scene_ids is None
                ):
                    raise TypeError(
                        "MUJOCO_COMMAND_TYPE_UNSUPPORTED:"
                        "RigidBodyKinematicCommand"
                    )
                if staged_object_kinematic is not None:
                    raise ValueError(
                        "MUJOCO_RIGID_BODY_KINEMATIC_DUPLICATE:object"
                    )
                staged_object_kinematic = (
                    self._validate_rigid_body_kinematic_command(command)
                )
            elif isinstance(command, JointEffortCommand):
                raise TypeError(
                    "MUJOCO_COMMAND_TYPE_UNSUPPORTED:"
                    f"{type(command).__name__}"
                )
            else:
                raise TypeError(
                    "MUJOCO_COMMAND_TYPE_UNSUPPORTED:"
                    f"{type(command).__name__}"
                )

        for ctrl_adr, target in staged.items():
            self._data.ctrl[ctrl_adr] = target
        if staged_object_kinematic is not None:
            if self._tabletop_scene_ids is None:
                raise RuntimeError("MUJOCO_TABLETOP_SCENE_IDS_UNAVAILABLE")
            self._data.eq_active[
                self._tabletop_scene_ids.object_weld_id
            ] = staged_object_kinematic

    def _apply_wam_bias_feedforward(self) -> None:
        if self._routing is None or self._data is None:
            raise RuntimeError("MUJOCO_BACKEND_ROUTING_UNAVAILABLE")
        for joint_name in self._robot.arm.joint_names:
            joint = self._routing.joint_by_name[joint_name]
            self._data.qfrc_applied[joint.qvel_adr] = (
                self._data.qfrc_bias[joint.qvel_adr]
            )

    def step(self, dt_s: float) -> None:
        self._require_initialized()
        if self._mujoco is None or self._model is None or self._data is None:
            raise RuntimeError("MUJOCO_BACKEND_RUNTIME_HANDLES_UNAVAILABLE")

        self._apply_wam_bias_feedforward()

        nstep = resolve_substeps(
            dt_s,
            self._cfg.physics_timestep_s,
            self._cfg.runtime_dt_tolerance_s,
        )
        before = float(self._data.time)
        self._mujoco.mj_step(
            self._model,
            self._data,
            nstep=nstep,
        )
        advanced = float(self._data.time) - before
        if abs(advanced - float(dt_s)) > self._cfg.runtime_dt_tolerance_s:
            raise RuntimeError(
                "MUJOCO_STEP_TIME_MISMATCH:"
                f"requested={float(dt_s)}:"
                f"advanced={advanced}:"
                f"substeps={nstep}"
            )

    def reset(self) -> None:
        self._require_initialized()
        self._reset_data_without_lifecycle_check()

    def shutdown(self) -> None:
        self._diagnostics["phase"] = "shutdown"
        self._clear_runtime_handles()
