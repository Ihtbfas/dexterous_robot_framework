from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping


class MuJoCoAssemblyError(ValueError):
    """Raised when WAM7 + L20 MuJoCo model assembly violates B1.2."""


@dataclass(frozen=True)
class MuJoCoCompositeModel:
    spec: Any
    model: Any
    data: Any
    metadata: Mapping[str, object]


def _mesh_names(mujoco: Any, model: Any) -> set[str]:
    result: set[str] = set()
    for idx in range(model.nmesh):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, idx)
        if name:
            result.add(name)
    return result


def _sanitize_hand_xml(
    hand_runtime: Path,
    *,
    reserved_mesh_names: set[str],
) -> tuple[str, dict[str, object]]:
    tree = ET.parse(hand_runtime)
    root = tree.getroot()
    if root.tag != "mujoco":
        raise MuJoCoAssemblyError("MUJOCO_HAND_ROOT_INVALID")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise MuJoCoAssemblyError("MUJOCO_HAND_WORLDBODY_MISSING")

    removed_floor = 0
    removed_lights = 0
    for child in list(worldbody):
        if child.tag == "geom" and child.get("name") == "floor":
            worldbody.remove(child)
            removed_floor += 1
        elif child.tag == "light":
            worldbody.remove(child)
            removed_lights += 1

    asset = root.find("asset")
    rename_map: dict[str, str] = {}
    rewritten_files: list[str] = []
    if asset is not None:
        used_names = set(reserved_mesh_names)
        for mesh in asset.findall("mesh"):
            name = mesh.get("name")
            file_ref = mesh.get("file")
            if not name or not file_ref:
                raise MuJoCoAssemblyError("MUJOCO_HAND_MESH_ENTRY_INVALID")

            if name in used_names:
                base = f"l20_asset__{name}"
                candidate = base
                counter = 2
                while candidate in used_names:
                    candidate = f"{base}_{counter}"
                    counter += 1
                rename_map[name] = candidate
                mesh.set("name", candidate)
                used_names.add(candidate)
            else:
                used_names.add(name)

            source = Path(file_ref)
            if not source.is_absolute():
                source = (hand_runtime.parent / source).resolve()
            if not source.is_file():
                raise MuJoCoAssemblyError(
                    f"MUJOCO_HAND_RUNTIME_MESH_MISSING:{file_ref}:{source}"
                )
            mesh.set("file", str(source))
            rewritten_files.append(str(source))

    if rename_map:
        for geom in root.findall(".//geom"):
            mesh_name = geom.get("mesh")
            if mesh_name in rename_map:
                geom.set("mesh", rename_map[mesh_name])

    return (
        ET.tostring(root, encoding="unicode"),
        {
            "removed_floor_geoms": removed_floor,
            "removed_world_lights": removed_lights,
            "renamed_mesh_assets": dict(sorted(rename_map.items())),
            "absolute_mesh_files": tuple(sorted(set(rewritten_files))),
        },
    )


def _resolve_flange_body(
    mujoco: Any,
    arm_model: Any,
    *,
    logical_flange: str,
) -> tuple[str, str]:
    direct_id = mujoco.mj_name2id(
        arm_model, mujoco.mjtObj.mjOBJ_BODY, logical_flange
    )
    if direct_id >= 0:
        return logical_flange, "direct_body_name"

    joint_name = "wam_j7_joint"
    joint_id = mujoco.mj_name2id(
        arm_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
    )
    if joint_id < 0:
        raise MuJoCoAssemblyError(
            f"MUJOCO_WAM_FLANGE_UNRESOLVED:{logical_flange}:{joint_name}"
        )
    body_id = int(arm_model.jnt_bodyid[joint_id])
    body_name = mujoco.mj_id2name(
        arm_model, mujoco.mjtObj.mjOBJ_BODY, body_id
    )
    if not body_name:
        raise MuJoCoAssemblyError("MUJOCO_WAM_FLANGE_BODY_NAME_MISSING")
    return body_name, "child_body_of_wam_j7_joint"


def _xyzw_to_wxyz(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion_xyzw
    return (w, x, y, z)


def _finite_tuple(values: tuple[float, ...], width: int, label: str) -> None:
    if len(values) != width or not all(math.isfinite(value) for value in values):
        raise MuJoCoAssemblyError(f"{label}_INVALID")


def assemble_wam7_l20_model(
    *,
    arm_runtime: str | Path,
    hand_runtime: str | Path,
    mount_position_xyz_m: tuple[float, float, float],
    mount_quaternion_xyzw: tuple[float, float, float, float],
    logical_wam_flange: str = "wam_j7",
    logical_l20_base: str = "l20_base",
    expected_physics_timestep_s: float | None = None,
) -> MuJoCoCompositeModel:
    import mujoco

    arm_path = Path(arm_runtime).resolve()
    hand_path = Path(hand_runtime).resolve()
    if not arm_path.is_file():
        raise MuJoCoAssemblyError(f"MUJOCO_WAM_RUNTIME_MISSING:{arm_path}")
    if not hand_path.is_file():
        raise MuJoCoAssemblyError(f"MUJOCO_L20_RUNTIME_MISSING:{hand_path}")

    _finite_tuple(mount_position_xyz_m, 3, "MUJOCO_MOUNT_POSITION")
    _finite_tuple(mount_quaternion_xyzw, 4, "MUJOCO_MOUNT_QUATERNION")

    arm_model_probe = mujoco.MjModel.from_xml_path(str(arm_path))
    flange_body_name, flange_policy = _resolve_flange_body(
        mujoco,
        arm_model_probe,
        logical_flange=logical_wam_flange,
    )
    reserved_mesh_names = _mesh_names(mujoco, arm_model_probe)

    hand_xml, sanitization = _sanitize_hand_xml(
        hand_path,
        reserved_mesh_names=reserved_mesh_names,
    )

    arm_spec = mujoco.MjSpec.from_file(str(arm_path))
    hand_spec = mujoco.MjSpec.from_string(hand_xml)

    flange_body = arm_spec.body(flange_body_name)
    if flange_body is None:
        raise MuJoCoAssemblyError(
            f"MUJOCO_WAM_FLANGE_SPEC_BODY_MISSING:{flange_body_name}"
        )

    if arm_spec.body(logical_l20_base) is not None:
        raise MuJoCoAssemblyError(
            f"MUJOCO_L20_BASE_NAME_COLLISION:{logical_l20_base}"
        )

    mount_quaternion_wxyz = _xyzw_to_wxyz(mount_quaternion_xyzw)
    mount_body = flange_body.add_body(
        name=logical_l20_base,
        pos=list(mount_position_xyz_m),
        quat=list(mount_quaternion_wxyz),
    )
    attach_frame = mount_body.add_frame(name="__l20_attach_frame")

    arm_spec.copy_during_attach = True
    arm_spec.attach(
        hand_spec,
        frame=attach_frame,
        prefix="",
        suffix="",
    )

    model = arm_spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    wam_expected = {
        "wam_j1_joint",
        "wam_j2_joint",
        "wam_j3_joint",
        "wam_j4_joint",
        "wam_j5_joint",
        "wam_j6_joint",
        "wam_j7_joint",
    }
    l20_expected = {
        "thumb_joint0", "thumb_joint1", "thumb_joint2",
        "thumb_joint3", "thumb_joint4",
        "index_joint0", "index_joint1", "index_joint2", "index_joint3",
        "middle_joint0", "middle_joint1", "middle_joint2", "middle_joint3",
        "ring_joint0", "ring_joint1", "ring_joint2", "ring_joint3",
        "little_joint0", "little_joint1", "little_joint2", "little_joint3",
    }
    joint_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, idx)
        for idx in range(model.njnt)
    }
    joint_names.discard(None)
    if joint_names != wam_expected | l20_expected:
        raise MuJoCoAssemblyError(
            "MUJOCO_COMPOSITE_JOINT_SET_MISMATCH:"
            f"missing={sorted((wam_expected | l20_expected) - joint_names)}:"
            f"extra={sorted(joint_names - (wam_expected | l20_expected))}"
        )
    if model.nq != 28 or model.nv != 28 or model.nu != 21:
        raise MuJoCoAssemblyError(
            f"MUJOCO_COMPOSITE_DIMENSION_INVALID:"
            f"nq={model.nq}:nv={model.nv}:nu={model.nu}"
        )

    l20_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, logical_l20_base
    )
    flange_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, flange_body_name
    )
    if l20_body_id < 0 or flange_body_id < 0:
        raise MuJoCoAssemblyError("MUJOCO_COMPOSITE_MOUNT_BODY_MISSING")
    if int(model.body_parentid[l20_body_id]) != flange_body_id:
        raise MuJoCoAssemblyError("MUJOCO_COMPOSITE_MOUNT_PARENT_INVALID")
    if int(model.body_jntnum[l20_body_id]) != 0:
        raise MuJoCoAssemblyError("MUJOCO_COMPOSITE_MOUNT_NOT_FIXED")

    actual_pos = tuple(float(value) for value in model.body_pos[l20_body_id])
    actual_quat_wxyz = tuple(
        float(value) for value in model.body_quat[l20_body_id]
    )
    if any(
        not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12)
        for a, b in zip(actual_pos, mount_position_xyz_m, strict=True)
    ):
        raise MuJoCoAssemblyError(
            f"MUJOCO_COMPOSITE_MOUNT_POSITION_MISMATCH:"
            f"expected={mount_position_xyz_m}:actual={actual_pos}"
        )
    if any(
        not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12)
        for a, b in zip(
            actual_quat_wxyz, mount_quaternion_wxyz, strict=True
        )
    ):
        raise MuJoCoAssemblyError(
            f"MUJOCO_COMPOSITE_MOUNT_QUATERNION_MISMATCH:"
            f"expected={mount_quaternion_wxyz}:actual={actual_quat_wxyz}"
        )

    floor_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )
    if floor_id >= 0:
        raise MuJoCoAssemblyError("MUJOCO_HAND_SCENE_FLOOR_LEAKED_INTO_ROBOT")

    actuator_targets: list[str] = []
    for aid in range(model.nu):
        jid = int(model.actuator_trnid[aid, 0])
        if jid < 0:
            raise MuJoCoAssemblyError(
                f"MUJOCO_COMPOSITE_ACTUATOR_TARGET_INVALID:{aid}"
            )
        name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_JOINT, jid
        )
        if not name:
            raise MuJoCoAssemblyError(
                f"MUJOCO_COMPOSITE_ACTUATOR_JOINT_NAME_MISSING:{aid}"
            )
        actuator_targets.append(name)
    if set(actuator_targets) != l20_expected or len(actuator_targets) != 21:
        raise MuJoCoAssemblyError(
            "MUJOCO_COMPOSITE_L20_ACTUATION_INVALID"
        )

    if expected_physics_timestep_s is not None:
        if not math.isclose(
            float(model.opt.timestep),
            float(expected_physics_timestep_s),
            rel_tol=0.0,
            abs_tol=1e-15,
        ):
            raise MuJoCoAssemblyError(
                f"MUJOCO_COMPOSITE_TIMESTEP_MISMATCH:"
                f"expected={expected_physics_timestep_s}:"
                f"actual={model.opt.timestep}"
            )

    metadata = MappingProxyType(
        {
            "logical_wam_flange": logical_wam_flange,
            "resolved_wam_flange_body": flange_body_name,
            "wam_flange_resolution_policy": flange_policy,
            "logical_l20_base": logical_l20_base,
            "expected_mount_position_xyz_m": mount_position_xyz_m,
            "expected_mount_quaternion_xyzw": mount_quaternion_xyzw,
            "compiled_mount_position_xyz_m": actual_pos,
            "compiled_mount_quaternion_wxyz": actual_quat_wxyz,
            "sanitization": sanitization,
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "njnt": int(model.njnt),
            "nbody": int(model.nbody),
            "ngeom": int(model.ngeom),
            "nlight": int(model.nlight),
            "physics_timestep_s": float(model.opt.timestep),
            "l20_actuator_targets": tuple(actuator_targets),
        }
    )
    return MuJoCoCompositeModel(
        spec=arm_spec,
        model=model,
        data=data,
        metadata=metadata,
    )
