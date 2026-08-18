from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS
from dexterous_robot.robots import ManipulatorSystem

from .config import IsaacBackendConfig, TabletopGraspLiftConfig


def build_articulation_candidate_patterns(
    default_prim_path: str | None,
    articulation_root_paths: Sequence[str],
    root_body_paths: Sequence[str],
) -> tuple[str, ...]:
    ordered: list[str] = []

    def add(value: str | None) -> None:
        if value and value not in ordered:
            ordered.append(value)

    # R15U runtime evidence proved the exact root rigid-body path is the
    # valid 28-DOF tensor authority while the top-level wildcard reports
    # BACKEND_NONE.  Prefer exact, evidence-backed paths first and keep the
    # wildcard only as a last-resort compatibility fallback.
    for path in root_body_paths:
        add(str(path))
    for path in articulation_root_paths:
        add(str(path))

    source_paths: list[str] = []
    if default_prim_path:
        source_paths.append(str(default_prim_path))
    source_paths.extend(str(path) for path in root_body_paths if path)
    source_paths.extend(str(path) for path in articulation_root_paths if path)
    for path in source_paths:
        parts = [part for part in path.split("/") if part]
        if parts:
            add("/" + parts[0] + "*")
    return tuple(ordered)


@dataclass(frozen=True)
class IsaacSceneHandles:
    stage: object
    session_layer: object
    original_edit_target: object
    articulation_candidates: tuple[str, ...]
    object_kinematic_attr: object


def _quatd_xyzw(Gf, values: Sequence[float]):
    x, y, z, w = (float(value) for value in values)
    return Gf.Quatd(w, Gf.Vec3d(x, y, z))


def _ensure_scope(UsdGeom, stage, path: str) -> None:
    parent = str(Path(path).parent).replace("\\", "/")
    if parent and parent != "/" and not stage.GetPrimAtPath(parent).IsValid():
        UsdGeom.Scope.Define(stage, parent)


def author_m1_scene(
    *,
    stage,
    app,
    l20_runtime: Path,
    robot: ManipulatorSystem,
    backend_config: IsaacBackendConfig,
    task_config: TabletopGraspLiftConfig,
) -> IsaacSceneHandles:  # pragma: no cover - requires Isaac runtime
    """Author the frozen M1 scene in the session layer before PhysX is loaded."""

    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    session = stage.GetSessionLayer()
    original = stage.GetEditTarget()
    stage.SetEditTarget(session)
    try:
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        if str(UsdGeom.GetStageUpAxis(stage)).upper() != "Z":
            raise RuntimeError("ISAAC_STAGE_UP_AXIS_AUTHORING_FAILED")

        wam_api_roots = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith("/wam_7dof") and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        if len(wam_api_roots) != 1:
            raise RuntimeError(f"ISAAC_WAM_ARTICULATION_ROOT_COUNT_INVALID:{wam_api_roots}")
        wam_root = stage.GetPrimAtPath(wam_api_roots[0])
        wam_physx = PhysxSchema.PhysxArticulationAPI.Get(stage, wam_root.GetPath())
        if not wam_physx:
            wam_physx = PhysxSchema.PhysxArticulationAPI.Apply(wam_root)
        wam_physx.GetEnabledSelfCollisionsAttr().Set(True)

        initial_by_name = dict(zip(robot.arm.joint_names, task_config.initial_wam_q_rad, strict=True))
        authored: set[str] = set()
        root_body_paths: list[str] = []
        for prim in stage.Traverse():
            name = prim.GetName()
            if name in initial_by_name and prim.IsA(UsdPhysics.RevoluteJoint):
                attr = prim.GetAttribute("state:angular:physics:position")
                if not attr or not attr.IsValid() or attr.Set(math.degrees(initial_by_name[name])) is not True:
                    raise RuntimeError(f"ISAAC_WAM_INITIAL_STATE_AUTHORING_FAILED:{name}")
                authored.add(name)
            if name == robot.arm.joint_names[0] and prim.IsA(UsdPhysics.RevoluteJoint):
                root_body_paths.extend(str(path) for path in UsdPhysics.Joint(prim).GetBody0Rel().GetTargets())
        if authored != set(robot.arm.joint_names):
            raise RuntimeError(f"ISAAC_WAM_INITIAL_STATE_JOINT_SET_INVALID:{sorted(authored)}")

        l20_xform = UsdGeom.Xform.Define(stage, backend_config.paths.l20_root)
        l20_prim = l20_xform.GetPrim()
        l20_prim.GetReferences().AddReference(str(l20_runtime))
        app.update()
        l20_prim = stage.GetPrimAtPath(backend_config.paths.l20_root)
        if not l20_prim or not l20_prim.IsValid() or not l20_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise RuntimeError("ISAAC_L20_REFERENCE_COMPOSITION_INVALID")
        xform = UsdGeom.Xformable(l20_prim)
        for op in xform.GetOrderedXformOps():
            op.GetAttr().Clear()
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*task_config.initial_l20_root_position_world_m))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            _quatd_xyzw(Gf, task_config.initial_l20_root_quaternion_xyzw)
        )

        old_world = stage.GetPrimAtPath(backend_config.paths.l20_world_fixed_joint)
        if not old_world or not old_world.IsValid():
            raise RuntimeError("ISAAC_L20_WORLD_FIXED_JOINT_NOT_FOUND")
        old_world.SetActive(False)
        l20_physx = PhysxSchema.PhysxArticulationAPI.Get(stage, l20_prim.GetPath())
        if not l20_physx:
            l20_physx = PhysxSchema.PhysxArticulationAPI.Apply(l20_prim)
        l20_physx.GetEnabledSelfCollisionsAttr().Set(False)

        _ensure_scope(UsdGeom, stage, backend_config.paths.integration_scope + "/placeholder")
        UsdGeom.Scope.Define(stage, backend_config.paths.integration_scope)
        mount = UsdPhysics.FixedJoint.Define(stage, backend_config.paths.integration_fixed_joint)
        mount.CreateBody0Rel().SetTargets([Sdf.Path(backend_config.paths.wam_j7_body)])
        mount.CreateBody1Rel().SetTargets([Sdf.Path(backend_config.paths.l20_base_body)])
        mount.CreateLocalPos0Attr(Gf.Vec3f(*robot.hand_mount.pose.position_xyz_m))
        mount.CreateLocalRot0Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        mount.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        mount.CreateLocalRot1Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

        hand_profile = backend_config.hand_open_hold
        for lane, name in enumerate(L20_PHYSICAL_JOINTS):
            joint_path = f"{backend_config.paths.l20_root}/Joints/{name}"
            prim = stage.GetPrimAtPath(joint_path)
            if not prim or not prim.IsValid():
                raise RuntimeError(f"ISAAC_L20_DRIVE_JOINT_INVALID:{name}")
            PhysxSchema.PhysxJointAPI.Apply(prim).GetMaxJointVelocityAttr().Set(
                hand_profile.max_joint_velocity_deg_s[lane]
            )
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            pairs = (
                (drive.GetTypeAttr(), hand_profile.drive_type),
                (drive.GetStiffnessAttr(), hand_profile.stiffness_usd_per_degree[lane]),
                (drive.GetDampingAttr(), hand_profile.damping_usd_per_degree_per_second[lane]),
                (drive.GetMaxForceAttr(), hand_profile.max_force_nm[lane]),
                (drive.GetTargetPositionAttr(), math.degrees(task_config.initial_hand_q_rad[lane])),
                (drive.GetTargetVelocityAttr(), 0.0),
            )
            for attr, value in pairs:
                if attr.Set(value) is not True:
                    raise RuntimeError(f"ISAAC_L20_DRIVE_AUTHORING_FAILED:{name}")

        scenes = [UsdPhysics.Scene(prim) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(scenes) > 1:
            raise RuntimeError("ISAAC_MULTIPLE_PHYSICS_SCENES")
        scene = scenes[0] if scenes else UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.GetGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).GetSolveArticulationContactLastAttr().Set(True)

        scene_root = str(Path(backend_config.paths.table).parent).replace("\\", "/")
        if scene_root and scene_root != "/":
            UsdGeom.Scope.Define(stage, scene_root)
        yaw = task_config.table_yaw_rad
        table_q = Gf.Quatd(math.cos(yaw / 2.0), Gf.Vec3d(0.0, 0.0, math.sin(yaw / 2.0)))
        table = UsdGeom.Cube.Define(stage, backend_config.paths.table)
        table.CreateSizeAttr(1.0)
        table.AddTranslateOp().Set(Gf.Vec3d(*task_config.table_center_world_m))
        table.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(table_q)
        table.AddScaleOp().Set(Gf.Vec3d(*task_config.table_dimensions_xyz_m))
        table_prim = table.GetPrim()
        UsdPhysics.CollisionAPI.Apply(table_prim).CreateCollisionEnabledAttr(True)

        object_cube = UsdGeom.Cube.Define(stage, backend_config.paths.object)
        object_cube.CreateSizeAttr(1.0)
        object_cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        object_cube.AddTranslateOp().Set(Gf.Vec3d(*task_config.object_position_world_m))
        object_cube.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(math.cos(task_config.object_yaw_rad / 2.0), Gf.Vec3d(0.0, 0.0, math.sin(task_config.object_yaw_rad / 2.0)))
        )
        object_cube.AddScaleOp().Set(Gf.Vec3d(*task_config.object_dimensions_xyz_m))
        object_prim = object_cube.GetPrim()
        rigid = UsdPhysics.RigidBodyAPI.Apply(object_prim)
        rigid.CreateRigidBodyEnabledAttr(True)
        object_kinematic_attr = rigid.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(object_prim).CreateMassAttr(task_config.object_mass_kg)
        UsdPhysics.CollisionAPI.Apply(object_prim).CreateCollisionEnabledAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(object_prim).CreateThresholdAttr().Set(0.0)

        scene_material = UsdShade.Material.Define(stage, backend_config.paths.scene_material)
        material_api = UsdPhysics.MaterialAPI.Apply(scene_material.GetPrim())
        material_api.CreateStaticFrictionAttr(task_config.object_static_friction)
        material_api.CreateDynamicFrictionAttr(task_config.object_dynamic_friction)
        material_api.CreateRestitutionAttr(task_config.object_restitution)
        UsdShade.MaterialBindingAPI.Apply(object_prim).Bind(scene_material, UsdShade.Tokens.weakerThanDescendants, "physics")
        UsdShade.MaterialBindingAPI.Apply(table_prim).Bind(scene_material, UsdShade.Tokens.weakerThanDescendants, "physics")

        grasp_material = UsdShade.Material.Define(stage, backend_config.paths.grasp_material)
        grasp_api = UsdPhysics.MaterialAPI.Apply(grasp_material.GetPrim())
        grasp_api.CreateStaticFrictionAttr(1.0)
        grasp_api.CreateDynamicFrictionAttr(1.0)
        grasp_api.CreateRestitutionAttr(0.0)
        bound = 0
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(backend_config.paths.l20_root) and prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(grasp_material, UsdShade.Tokens.strongerThanDescendants, "physics")
                bound += 1
        if bound == 0:
            raise RuntimeError("ISAAC_GRASP_MATERIAL_NO_L20_COLLISION_PRIMS")

        default_prim = stage.GetDefaultPrim()
        default_prim_path = str(default_prim.GetPath()) if default_prim and default_prim.IsValid() else None
        candidates = build_articulation_candidate_patterns(default_prim_path, wam_api_roots, root_body_paths)
        return IsaacSceneHandles(
            stage=stage,
            session_layer=session,
            original_edit_target=original,
            articulation_candidates=candidates,
            object_kinematic_attr=object_kinematic_attr,
        )
    finally:
        stage.SetEditTarget(original)
