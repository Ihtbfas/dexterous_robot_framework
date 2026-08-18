from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Sequence

from dexterous_robot.backends.base import Backend, BackendState, Command
from dexterous_robot.config import LocalAssetConfig
from dexterous_robot.core import JointEffortCommand, JointPositionCommand, JointState, Pose, RigidBodyKinematicCommand
from dexterous_robot.robots import ManipulatorSystem

from .config import IsaacBackendConfig, TabletopGraspLiftConfig
from .contacts import IsaacContactCollector, summarize_contacts
from .topology import JointRouting, build_joint_routing
from .transform_sync import RootSeedDynamicTransformPolicy, compare_position_sources


class IsaacBackend(Backend):
    """Isaac/PhysX adapter with all simulator imports delayed until initialize()."""

    def __init__(
        self,
        *,
        robot: ManipulatorSystem,
        backend_config: IsaacBackendConfig,
        task_config: TabletopGraspLiftConfig,
        assets: LocalAssetConfig,
        headless: bool,
    ) -> None:
        if not isinstance(robot, ManipulatorSystem):
            raise TypeError("ISAAC_BACKEND_ROBOT_INVALID")
        if not isinstance(backend_config, IsaacBackendConfig):
            raise TypeError("ISAAC_BACKEND_CONFIG_INVALID")
        if not isinstance(task_config, TabletopGraspLiftConfig):
            raise TypeError("ISAAC_TASK_CONFIG_INVALID")
        if not isinstance(assets, LocalAssetConfig):
            raise TypeError("ISAAC_ASSET_CONFIG_INVALID")
        self._robot = robot
        self._cfg = backend_config
        self._task = task_config
        self._assets = assets
        self._headless = bool(headless)
        self._initialized = False
        self._app = None
        self._physx = None
        self._simulation_manager = None
        self._physics_view = None
        self._stage = None
        self._usd_context = None
        self._articulation = None
        self._routing: JointRouting | None = None
        self._j7_view = None
        self._hand_base_view = None
        self._object_view = None
        self._contact_collector: IsaacContactCollector | None = None
        self._transform_policy: RootSeedDynamicTransformPolicy | None = None
        self._session_layer = None
        self._object_kinematic_attr = None
        self._current_position_targets: tuple[float, ...] | None = None
        self._active_arm_profile: str | None = None
        self._active_hand_profile: str | None = None
        self._sim_time_s = 0.0
        self._diagnostics: dict[str, object] = {}

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def diagnostics(self) -> dict[str, object]:
        return dict(self._diagnostics)

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _verify_assets(self) -> dict[str, str]:
        rows = {
            "wam_runtime": (Path(self._assets.wam_runtime), self._cfg.asset_authority.wam_runtime_sha256),
            "l20_runtime": (Path(self._assets.l20_runtime), self._cfg.asset_authority.l20_runtime_sha256),
        }
        hashes: dict[str, str] = {}
        for label, (path, expected) in rows.items():
            if not path.is_file():
                raise RuntimeError(f"ISAAC_ASSET_MISSING:{label}:{path}")
            actual = self._sha256(path)
            if actual != expected:
                raise RuntimeError(f"ISAAC_ASSET_SHA256_MISMATCH:{label}:{actual}:{expected}")
            hashes[label] = actual
        return hashes

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("ISAAC_BACKEND_NOT_INITIALIZED")

    def initialize(self) -> None:  # pragma: no cover - requires Isaac runtime
        if self._initialized:
            raise RuntimeError("ISAAC_BACKEND_ALREADY_INITIALIZED")
        self._diagnostics = {"phase": "asset_verification"}
        asset_hashes = self._verify_assets()
        self._diagnostics.update({"phase": "kit_starting", "asset_hashes_before": dict(asset_hashes)})
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": self._headless,
                "multi_gpu": False,
                "disable_viewport_updates": bool(self._headless),
                "limit_cpu_threads": 8,
                "sync_loads": True,
                "enable_crashreporter": False,
                "width": 1280,
                "height": 800,
            }
        )
        self._app = app
        self._diagnostics["phase"] = "kit_started"
        try:
            self._initialize_after_kit(asset_hashes)
            # Lifecycle contract: initialized becomes true only after a coherent
            # state can actually be sampled.
            self._initialized = True
            self._diagnostics["phase"] = "initial_state_sampling"
            self.read_state()
            self._diagnostics["phase"] = "ready"
        except Exception as exc:
            self._initialized = False
            self._diagnostics["failure"] = f"{type(exc).__name__}:{exc}"
            # Do not close SimulationApp here.  Some Kit runtimes terminate the
            # interpreter while close() is running, which previously erased the
            # outer smoke runner's exception receipt.  The caller owns cleanup
            # after it has persisted diagnostics.
            raise

    def _initialize_after_kit(self, asset_hashes: dict[str, str]) -> None:  # pragma: no cover - requires Isaac runtime
        import time

        import numpy as np
        import omni.usd
        import warp as wp
        from isaacsim.core.simulation_manager import SimulationManager
        from omni.physx import get_physx_interface
        from omni.physics.core import get_physics_simulation_interface
        from pxr import PhysicsSchemaTools

        from .scene import author_m1_scene

        self._diagnostics["phase"] = "stage_open_request"
        context = omni.usd.get_context()
        if not context.open_stage(str(self._assets.wam_runtime)):
            raise RuntimeError("ISAAC_WAM_STAGE_OPEN_REQUEST_FAILED")
        deadline = time.monotonic() + self._cfg.stage_load_timeout_s
        while context.get_stage_loading_status()[2] > 0:
            if time.monotonic() >= deadline:
                raise TimeoutError("ISAAC_WAM_STAGE_LOAD_TIMEOUT")
            self._app.update()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("ISAAC_WAM_STAGE_NULL")
        self._diagnostics["phase"] = "stage_loaded"
        self._stage = stage
        self._usd_context = context

        handles = author_m1_scene(
            stage=stage,
            app=self._app,
            l20_runtime=Path(self._assets.l20_runtime),
            robot=self._robot,
            backend_config=self._cfg,
            task_config=self._task,
        )

        self._session_layer = handles.session_layer
        self._object_kinematic_attr = handles.object_kinematic_attr
        self._diagnostics.update({
            "phase": "scene_authored",
            "articulation_candidates": list(handles.articulation_candidates),
        })
        physx = get_physx_interface()
        manager = SimulationManager()
        physx.force_load_physics_from_usd()
        manager.initialize_physics()
        physics_view = manager.get_physics_simulation_view()
        self._physx = physx
        self._simulation_manager = manager
        self._physics_view = physics_view
        self._diagnostics["phase"] = "physics_initialized"

        policy = RootSeedDynamicTransformPolicy(
            prim_path=self._cfg.paths.object,
            update_to_fast_cache=self._cfg.transform_sync.update_to_fast_cache,
            update_to_usd=self._cfg.transform_sync.update_to_usd,
            tolerance_m=self._cfg.transform_sync.position_tolerance_m,
        )
        policy.sync(physx)
        release_receipt = policy.seed_root_then_release_session(
            stage=stage,
            session_layer=handles.session_layer,
            physx=physx,
            app=self._app,
            headless=self._headless,
        )
        self._transform_policy = policy
        self._diagnostics.update({"phase": "transform_release_complete", "transform_release": release_receipt})

        expected_names = tuple(self._robot.arm.joint_names) + tuple(
            # resolver verifies the actual backend order by set and later maps by name
            ()
        )
        del expected_names
        articulation, selected_pattern, attempts = self._resolve_combined_articulation(physics_view, handles.articulation_candidates)
        self._articulation = articulation
        self._diagnostics.update({
            "phase": "articulation_resolved",
            "combined_articulation_attempts": attempts,
            "selected_articulation_pattern": selected_pattern,
        })
        actual_names = tuple(str(path).split("/")[-1] for path in articulation.dof_paths[0])
        routing = build_joint_routing(actual_names, self._robot)
        self._routing = routing

        self._j7_view = physics_view.create_rigid_body_view(self._cfg.paths.wam_j7_body)
        self._hand_base_view = physics_view.create_rigid_body_view(self._cfg.paths.l20_base_body)
        self._object_view = physics_view.create_rigid_body_view(self._cfg.paths.object)
        for label, view in (("j7", self._j7_view), ("hand_base", self._hand_base_view), ("object", self._object_view)):
            if getattr(view, "_backend", None) is None or int(view.count) != 1:
                raise RuntimeError(f"ISAAC_RIGID_BODY_VIEW_INVALID:{label}")
        self._diagnostics["phase"] = "rigid_body_views_resolved"

        full_initial = routing.scatter(
            arm_values=self._task.initial_wam_q_rad,
            hand_values=self._task.initial_hand_q_rad,
        )
        row = wp.array([list(full_initial)], dtype=wp.float32, device="cpu")
        zeros = wp.array([[0.0] * 28], dtype=wp.float32, device="cpu")
        indices = wp.array([0], dtype=wp.int32, device="cpu")
        articulation.set_dof_positions(row, indices)
        articulation.set_dof_velocities(zeros, indices)
        articulation.set_dof_actuation_forces(zeros, indices)
        articulation.set_dof_position_targets(row, indices)
        physics_view.update_articulations_kinematic()
        policy.sync(physx)
        self._current_position_targets = full_initial
        self._active_arm_profile = None
        self._active_hand_profile = None
        self._apply_hand_drive_profile("hand_open_hold", wp, indices)
        self._active_hand_profile = "hand_open_hold"
        self._diagnostics["phase"] = "initial_dof_state_applied"

        collector = IsaacContactCollector(
            physics_schema_tools=PhysicsSchemaTools,
            simulation_interface=get_physics_simulation_interface(),
            object_path=self._cfg.paths.object,
        )
        collector.subscribe()
        self._contact_collector = collector
        self._diagnostics["phase"] = "contact_collector_subscribed"

        # M1-R5 deliberately has no motion task yet.  Keep transform validation
        # passive here: R15U proved the production writeback path by sampling
        # normal physics-driven motion after update_transformations().  A tensor
        # set_transforms teleport is a different API contract and is not part of
        # the approved R5 smoke specification.  The smoke runner records the
        # four-way checkpoint after its ten ordinary RuntimeSession cycles.
        self._diagnostics = {
            "asset_hashes_before": asset_hashes,
            "combined_articulation": {
                "selected_pattern": selected_pattern,
                "count": int(articulation.count),
                "max_dofs": int(articulation.max_dofs),
                "backend_joint_names": list(actual_names),
                "arm_backend_indices": list(routing.arm_backend_indices),
                "hand_backend_indices": list(routing.hand_backend_indices),
                "attempts": attempts,
            },
            "transform_release": release_receipt,
            "transform_checkpoints": [],
            "hand_tcp_source_prim": self._cfg.paths.l20_base_body,
            "hand_tcp_policy": "M1_TCP_COINCIDENT_WITH_L20_BASE",
        }

    def _resolve_combined_articulation(self, physics_view, candidates):  # pragma: no cover - requires Isaac runtime
        expected_set = set(self._robot.arm.joint_names) | set(self._robot.hand.physical_joints)
        attempts: list[dict[str, object]] = []
        for pattern in candidates:
            try:
                view = physics_view.create_articulation_view(pattern)
            except Exception as exc:
                attempts.append({"pattern": pattern, "accepted": False, "reason": f"CREATE_EXCEPTION:{type(exc).__name__}:{exc}"})
                continue
            if getattr(view, "_backend", None) is None:
                attempts.append({"pattern": pattern, "accepted": False, "reason": "BACKEND_NONE"})
                continue
            try:
                count = int(view.count)
                max_dofs = int(view.max_dofs)
                names = tuple(str(path).split("/")[-1] for path in view.dof_paths[0]) if count else ()
            except Exception as exc:
                attempts.append({"pattern": pattern, "accepted": False, "reason": f"READBACK_EXCEPTION:{type(exc).__name__}:{exc}"})
                continue
            accepted = count == 1 and max_dofs == 28 and len(names) == 28 and set(names) == expected_set
            attempts.append({"pattern": pattern, "accepted": accepted, "count": count, "max_dofs": max_dofs, "dof_names": list(names)})
            if accepted:
                return view, pattern, attempts
        raise RuntimeError(f"ISAAC_COMBINED_ARTICULATION_RESOLUTION_FAILED:{attempts}")

    @staticmethod
    def _numpy_row(tensor) -> tuple[float, ...]:  # pragma: no cover - requires Isaac runtime
        values = tensor.numpy().reshape(-1)
        return tuple(float(value) for value in values)

    @staticmethod
    def _pose_from_rigid_view(view, *, frame_id: str) -> Pose:  # pragma: no cover - requires Isaac runtime
        values = view.get_transforms().numpy().reshape(-1, 7)[0]
        return Pose(
            tuple(float(x) for x in values[:3]),
            tuple(float(x) for x in values[3:7]),
            frame_id,
        )

    def read_state(self) -> BackendState:
        self._require_initialized()
        assert self._articulation is not None and self._routing is not None
        assert self._hand_base_view is not None and self._object_view is not None
        q = self._numpy_row(self._articulation.get_dof_positions())
        dq = self._numpy_row(self._articulation.get_dof_velocities())
        arm_q = self._routing.gather_arm(q)
        arm_dq = self._routing.gather_arm(dq)
        hand_q = self._routing.gather_hand(q)
        hand_dq = self._routing.gather_hand(dq)
        samples = self._contact_collector.snapshot() if self._contact_collector is not None else ()
        contacts = summarize_contacts(
            samples,
            dt_s=self._cfg.physics_dt_s,
            object_path=self._cfg.paths.object,
            table_path=self._cfg.paths.table,
        )
        return BackendState(
            device_states={
                self._robot.arm.device_id: JointState(tuple(self._robot.arm.joint_names), arm_q, arm_dq),
                self._robot.hand.device_id: JointState(tuple(self._robot.hand.physical_joints), hand_q, hand_dq),
            },
            body_poses={
                "hand_tcp": self._pose_from_rigid_view(self._hand_base_view, frame_id=self._task.world_frame),
                "object": self._pose_from_rigid_view(self._object_view, frame_id=self._task.world_frame),
            },
            signals={
                "object_table_normal_n": contacts.object_table_normal_n,
                "opposing_y_squeeze_n": contacts.opposing_y_squeeze_n,
            },
        )

    def apply(self, commands: Sequence[Command]) -> None:
        self._require_initialized()
        assert self._articulation is not None and self._routing is not None
        if self._current_position_targets is None:
            raise RuntimeError("ISAAC_POSITION_TARGET_CACHE_UNAVAILABLE")
        import warp as wp  # lazy: only reachable after initialize

        current = list(self._current_position_targets)
        touched_position = False
        effort = [0.0] * 28
        touched_effort = False
        arm_profile_requested: str | None = None
        hand_profile_requested: str | None = None
        for command in commands:
            if isinstance(command, JointPositionCommand):
                if command.device_id == self._robot.arm.device_id:
                    if command.joint_names != tuple(self._robot.arm.joint_names):
                        raise ValueError("ISAAC_ARM_COMMAND_JOINT_ORDER_INVALID")
                    if command.profile not in (None, "arm_carry_position_drive"):
                        raise ValueError("ISAAC_ARM_COMMAND_PROFILE_INVALID")
                    for index, value in zip(self._routing.arm_backend_indices, command.position_rad, strict=True):
                        current[index] = value
                    if command.profile is not None:
                        arm_profile_requested = command.profile
                elif command.device_id == self._robot.hand.device_id:
                    if command.joint_names != tuple(self._robot.hand.physical_joints):
                        raise ValueError("ISAAC_HAND_COMMAND_JOINT_ORDER_INVALID")
                    if command.profile not in (None, "hand_open_hold", "hand_grasp_lock"):
                        raise ValueError("ISAAC_HAND_COMMAND_PROFILE_INVALID")
                    for index, value in zip(self._routing.hand_backend_indices, command.position_rad, strict=True):
                        current[index] = value
                    if command.profile is not None:
                        hand_profile_requested = command.profile
                else:
                    raise ValueError(f"ISAAC_COMMAND_DEVICE_UNKNOWN:{command.device_id}")
                touched_position = True
            elif isinstance(command, JointEffortCommand):
                if command.device_id == self._robot.arm.device_id:
                    if command.joint_names != tuple(self._robot.arm.joint_names):
                        raise ValueError("ISAAC_ARM_COMMAND_JOINT_ORDER_INVALID")
                    for index, value in zip(self._routing.arm_backend_indices, command.effort_nm, strict=True):
                        effort[index] = value
                elif command.device_id == self._robot.hand.device_id:
                    if command.joint_names != tuple(self._robot.hand.physical_joints):
                        raise ValueError("ISAAC_HAND_COMMAND_JOINT_ORDER_INVALID")
                    for index, value in zip(self._routing.hand_backend_indices, command.effort_nm, strict=True):
                        effort[index] = value
                else:
                    raise ValueError(f"ISAAC_COMMAND_DEVICE_UNKNOWN:{command.device_id}")
                touched_effort = True
            elif isinstance(command, RigidBodyKinematicCommand):
                if command.body_id != "object":
                    raise ValueError(f"ISAAC_RIGID_BODY_COMMAND_UNKNOWN:{command.body_id}")
                self._set_object_kinematic(command.kinematic_enabled)
            else:
                raise TypeError("ISAAC_COMMAND_TYPE_UNSUPPORTED")

        indices = wp.array([0], dtype=wp.int32, device="cpu")
        if touched_position:
            row = wp.array([current], dtype=wp.float32, device="cpu")
            self._articulation.set_dof_position_targets(row, indices)
            self._current_position_targets = tuple(current)
        if touched_effort:
            self._articulation.set_dof_actuation_forces(wp.array([effort], dtype=wp.float32, device="cpu"), indices)
        if arm_profile_requested is not None and arm_profile_requested != self._active_arm_profile:
            self._apply_arm_carry_drive_profile(wp, indices)
            self._active_arm_profile = arm_profile_requested
        if hand_profile_requested is not None and hand_profile_requested != self._active_hand_profile:
            self._apply_hand_drive_profile(hand_profile_requested, wp, indices)
            self._active_hand_profile = hand_profile_requested

    def _set_object_kinematic(self, enabled: bool) -> None:  # pragma: no cover - requires Isaac runtime
        if self._stage is None or self._session_layer is None or self._object_kinematic_attr is None:
            raise RuntimeError("ISAAC_OBJECT_KINEMATIC_HANDLE_UNAVAILABLE")
        original = self._stage.GetEditTarget()
        try:
            self._stage.SetEditTarget(self._session_layer)
            if self._object_kinematic_attr.Set(bool(enabled)) is not True:
                raise RuntimeError("ISAAC_OBJECT_KINEMATIC_WRITE_FAILED")
        finally:
            self._stage.SetEditTarget(original)
        self._diagnostics["object_kinematic_enabled"] = bool(enabled)
        transitions = self._diagnostics.setdefault("object_kinematic_transitions", [])
        if isinstance(transitions, list):
            transitions.append({"simulation_time_s": float(self._sim_time_s), "kinematic_enabled": bool(enabled)})

    def _apply_arm_carry_drive_profile(self, wp, indices) -> None:  # pragma: no cover - requires Isaac runtime
        assert self._routing is not None and self._articulation is not None
        stiffness = list(self._numpy_row(self._articulation.get_dof_stiffnesses()))
        damping = list(self._numpy_row(self._articulation.get_dof_dampings()))
        max_force = list(self._numpy_row(self._articulation.get_dof_max_forces()))
        profile = self._cfg.arm_carry_position_drive
        for lane, index in enumerate(self._routing.arm_backend_indices):
            stiffness[index] = profile.stiffness[lane]
            damping[index] = profile.damping[lane]
            max_force[index] = profile.max_force[lane]
        self._articulation.set_dof_stiffnesses(wp.array([stiffness], dtype=wp.float32, device="cpu"), indices)
        self._articulation.set_dof_dampings(wp.array([damping], dtype=wp.float32, device="cpu"), indices)
        self._articulation.set_dof_max_forces(wp.array([max_force], dtype=wp.float32, device="cpu"), indices)

    def _apply_hand_drive_profile(self, profile_name: str, wp, indices) -> None:  # pragma: no cover - requires Isaac runtime
        assert self._routing is not None and self._articulation is not None
        if profile_name == "hand_open_hold":
            profile = self._cfg.hand_open_hold
        elif profile_name == "hand_grasp_lock":
            profile = self._cfg.hand_grasp_lock
        else:
            raise ValueError(f"ISAAC_HAND_PROFILE_UNKNOWN:{profile_name}")
        stiffness = list(self._numpy_row(self._articulation.get_dof_stiffnesses()))
        damping = list(self._numpy_row(self._articulation.get_dof_dampings()))
        max_force = list(self._numpy_row(self._articulation.get_dof_max_forces()))
        # USD angular DriveAPI is authored per degree; tensor articulation APIs
        # expose the PhysX per-radian values observed in the frozen runtime.
        usd_to_backend = 180.0 / math.pi
        for lane, index in enumerate(self._routing.hand_backend_indices):
            stiffness[index] = profile.stiffness_usd_per_degree[lane] * usd_to_backend
            damping[index] = profile.damping_usd_per_degree_per_second[lane] * usd_to_backend
            max_force[index] = profile.max_force_nm[lane]
        self._articulation.set_dof_stiffnesses(wp.array([stiffness], dtype=wp.float32, device="cpu"), indices)
        self._articulation.set_dof_dampings(wp.array([damping], dtype=wp.float32, device="cpu"), indices)
        self._articulation.set_dof_max_forces(wp.array([max_force], dtype=wp.float32, device="cpu"), indices)
        self._diagnostics["active_hand_profile"] = profile_name

    def step(self, dt_s: float) -> None:
        self._require_initialized()
        dt = float(dt_s)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("ISAAC_BACKEND_DT_INVALID")
        if abs(dt - self._cfg.physics_dt_s) > 1e-12:
            raise ValueError(f"ISAAC_BACKEND_DT_MISMATCH:{dt}:{self._cfg.physics_dt_s}")
        assert self._physx is not None and self._transform_policy is not None
        if self._contact_collector is not None:
            self._contact_collector.clear()
        self._sim_time_s += dt
        self._physx.update_simulation(dt, self._sim_time_s)
        self._transform_policy.sync(self._physx)
        if self._app is not None and not self._headless:
            self._app.update()

    def reset(self) -> None:
        self._require_initialized()
        assert self._routing is not None and self._articulation is not None
        import warp as wp

        full_initial = self._routing.scatter(arm_values=self._task.initial_wam_q_rad, hand_values=self._task.initial_hand_q_rad)
        row = wp.array([list(full_initial)], dtype=wp.float32, device="cpu")
        zeros = wp.array([[0.0] * 28], dtype=wp.float32, device="cpu")
        indices = wp.array([0], dtype=wp.int32, device="cpu")
        self._articulation.set_dof_positions(row, indices)
        self._articulation.set_dof_velocities(zeros, indices)
        self._articulation.set_dof_actuation_forces(zeros, indices)
        self._articulation.set_dof_position_targets(row, indices)
        self._physics_view.update_articulations_kinematic()
        self._transform_policy.sync(self._physx)
        self._sim_time_s = 0.0
        self._current_position_targets = full_initial
        self._active_arm_profile = None
        self._active_hand_profile = None
        self._apply_hand_drive_profile("hand_open_hold", wp, indices)
        self._active_hand_profile = "hand_open_hold"
        self._set_object_kinematic(True)

    def _runtime_positions(self, *, stage, context, np) -> dict[str, tuple[float, float, float] | None]:  # pragma: no cover
        assert self._object_view is not None and self._transform_policy is not None and self._physx is not None
        from pxr import Usd, UsdGeom

        tensor = np.asarray(self._object_view.get_transforms().numpy(), dtype=float).reshape(-1, 7)[0]
        physx_position, _ = self._transform_policy.direct_physx_pose(self._physx)
        prim = stage.GetPrimAtPath(self._cfg.paths.object)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        usd_position = tuple(float(x) for x in matrix.ExtractTranslation())
        fabric_position = None
        try:
            from usdrt import Rt as RtGeom
            from usdrt import Usd as RtUsd

            rt_stage = RtUsd.Stage.Attach(context.get_stage_id())
            rt_prim = rt_stage.GetPrimAtPath(self._cfg.paths.object)
            rt_matrix = RtGeom.Xformable(rt_prim).GetFabricHierarchyWorldMatrixAttr().Get()
            translation = rt_matrix.ExtractTranslation()
            fabric_position = tuple(float(x) for x in translation)
        except Exception:
            fabric_position = None
        return {
            "tensor": tuple(float(x) for x in tensor[:3]),
            "physx": physx_position,
            "usd": usd_position,
            "fabric": fabric_position,
        }

    def capture_transform_checkpoint(self, label: str) -> dict[str, object]:  # pragma: no cover - requires Isaac runtime
        """Passively sample the dynamic object through all four runtime paths.

        This method never teleports or otherwise commands the object.  Callers
        choose meaningful checkpoints after normal RuntimeSession cycles.
        """
        self._require_initialized()
        if not isinstance(label, str) or not label.strip():
            raise ValueError("ISAAC_TRANSFORM_CHECKPOINT_LABEL_INVALID")
        assert self._stage is not None and self._usd_context is not None
        assert self._transform_policy is not None and self._physx is not None
        import numpy as np

        self._transform_policy.sync(self._physx)
        if self._app is not None and not self._headless:
            self._app.update()
        positions = self._runtime_positions(stage=self._stage, context=self._usd_context, np=np)
        audit = compare_position_sources(
            tensor_xyz=positions["tensor"],
            physx_xyz=positions["physx"],
            usd_xyz=positions["usd"],
            fabric_xyz=positions["fabric"],
            tolerance_m=self._cfg.transform_sync.position_tolerance_m,
        )
        row = {
            "label": label.strip(),
            "simulation_time_s": float(self._sim_time_s),
            "positions": dict(audit.positions),
            "max_pairwise_position_error_m": audit.max_pairwise_position_error_m,
            "consistent": audit.consistent,
        }
        checkpoints = self._diagnostics.setdefault("transform_checkpoints", [])
        if not isinstance(checkpoints, list):
            raise RuntimeError("ISAAC_TRANSFORM_CHECKPOINT_STORAGE_INVALID")
        checkpoints.append(row)
        self._diagnostics["last_transform_checkpoint"] = row

        # Persist the sampled row before gating so a failure remains diagnosable
        # in the sealed receipt.
        if positions["fabric"] is None:
            self._diagnostics["transform_checkpoint_failure"] = "FABRIC_UNAVAILABLE"
            raise RuntimeError("ISAAC_FABRIC_TRANSFORM_CHECKPOINT_UNAVAILABLE")
        if not audit.consistent:
            self._diagnostics["transform_checkpoint_failure"] = "POSITION_INCONSISTENT"
            raise RuntimeError("ISAAC_TRANSFORM_CHECKPOINT_INCONSISTENT")
        self._diagnostics.pop("transform_checkpoint_failure", None)
        return row

    def shutdown(self, force: bool = False) -> None:
        if not self._initialized and not force and self._app is None:
            return
        app = self._app
        self._initialized = False
        self._contact_collector = None
        self._articulation = None
        self._routing = None
        self._physics_view = None
        self._stage = None
        self._usd_context = None
        self._physx = None
        self._simulation_manager = None
        self._j7_view = None
        self._hand_base_view = None
        self._object_view = None
        self._transform_policy = None
        self._current_position_targets = None
        self._app = None
        if app is not None:
            app.close()
