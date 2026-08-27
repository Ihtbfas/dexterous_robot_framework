from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

from dexterous_robot.config.tasks import TabletopGraspLiftConfig


_TABLE_GEOM_NAME = "tabletop_table_geom"
_OBJECT_BODY_NAME = "tabletop_object"
_OBJECT_GEOM_NAME = "tabletop_object_geom"
_OBJECT_FREE_JOINT_NAME = "tabletop_object_free"
_OBJECT_ANCHOR_BODY_NAME = "tabletop_object_anchor"
_OBJECT_WELD_NAME = "tabletop_object_weld"

_EXTERNAL_CONTYPE = 1
_EXTERNAL_CONAFFINITY = 1
_DEFAULT_ROLLING_FRICTION = 0.005
_DEFAULT_TORSIONAL_FRICTION = 0.0001
_CONTRACT_TOLERANCE = 1.0e-12


@dataclass(frozen=True)
class MuJoCoTabletopSceneHandles:
    table_geom_name: str
    object_body_name: str
    object_geom_name: str
    object_free_joint_name: str
    object_anchor_body_name: str
    object_weld_name: str
    object_initial_position_world_m: tuple[float, float, float]
    object_initial_quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class MuJoCoTabletopSceneIds:
    table_geom_id: int
    object_body_id: int
    object_geom_id: int
    object_free_joint_id: int
    object_anchor_body_id: int
    object_weld_id: int


def _named_id(
    mujoco: Any,
    model: Any,
    objtype: Any,
    name: str,
) -> int:
    value = int(mujoco.mj_name2id(model, objtype, name))
    if value < 0:
        raise RuntimeError(f"MUJOCO_TABLETOP_RUNTIME_ID_UNRESOLVED:{name}")
    return value


def resolve_tabletop_scene_ids(
    *,
    mujoco: Any,
    model: Any,
    handles: MuJoCoTabletopSceneHandles,
) -> MuJoCoTabletopSceneIds:
    if not isinstance(handles, MuJoCoTabletopSceneHandles):
        raise TypeError("MUJOCO_TABLETOP_SCENE_HANDLES_INVALID")
    return MuJoCoTabletopSceneIds(
        table_geom_id=_named_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            handles.table_geom_name,
        ),
        object_body_id=_named_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            handles.object_body_name,
        ),
        object_geom_id=_named_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            handles.object_geom_name,
        ),
        object_free_joint_id=_named_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            handles.object_free_joint_name,
        ),
        object_anchor_body_id=_named_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            handles.object_anchor_body_name,
        ),
        object_weld_id=_named_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_EQUALITY,
            handles.object_weld_name,
        ),
    )


def box_half_extents(
    dimensions_xyz_m: Sequence[float],
) -> tuple[float, float, float]:
    values = tuple(float(value) for value in dimensions_xyz_m)
    if (
        len(values) != 3
        or not all(math.isfinite(value) and value > 0.0 for value in values)
    ):
        raise ValueError("MUJOCO_TABLETOP_BOX_DIMENSIONS_INVALID")
    return tuple(value / 2.0 for value in values)


def yaw_quaternion_wxyz(
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    yaw = float(yaw_rad)
    if not math.isfinite(yaw):
        raise ValueError("MUJOCO_TABLETOP_YAW_INVALID")
    half = yaw / 2.0
    return (
        math.cos(half),
        0.0,
        0.0,
        math.sin(half),
    )


def _require_available_name(spec: Any, accessor: str, name: str) -> None:
    resolver = getattr(spec, accessor)
    if resolver(name) is not None:
        raise RuntimeError(
            f"MUJOCO_TABLETOP_NAME_COLLISION:{accessor}:{name}"
        )


def _validate_task_contract(
    task_config: TabletopGraspLiftConfig,
) -> None:
    if not isinstance(task_config, TabletopGraspLiftConfig):
        raise TypeError("MUJOCO_TABLETOP_TASK_CONFIG_INVALID")
    if task_config.world_frame != "world":
        raise ValueError(
            "MUJOCO_TABLETOP_WORLD_FRAME_INVALID:"
            f"{task_config.world_frame}"
        )

    table_half = box_half_extents(task_config.table_dimensions_xyz_m)
    object_half = box_half_extents(task_config.object_dimensions_xyz_m)

    authored_table_top = (
        float(task_config.table_center_world_m[2]) + table_half[2]
    )
    if not math.isclose(
        authored_table_top,
        float(task_config.table_top_world_z_m),
        rel_tol=0.0,
        abs_tol=_CONTRACT_TOLERANCE,
    ):
        raise ValueError(
            "MUJOCO_TABLETOP_TABLE_TOP_CONTRACT_MISMATCH:"
            f"derived={authored_table_top}:"
            f"declared={task_config.table_top_world_z_m}"
        )

    authored_object_bottom = (
        float(task_config.object_position_world_m[2]) - object_half[2]
    )
    if not math.isclose(
        authored_object_bottom,
        float(task_config.table_top_world_z_m),
        rel_tol=0.0,
        abs_tol=_CONTRACT_TOLERANCE,
    ):
        raise ValueError(
            "MUJOCO_TABLETOP_OBJECT_REST_CONTRACT_MISMATCH:"
            f"object_bottom={authored_object_bottom}:"
            f"table_top={task_config.table_top_world_z_m}"
        )

    if not math.isclose(
        float(task_config.object_static_friction),
        float(task_config.object_dynamic_friction),
        rel_tol=0.0,
        abs_tol=_CONTRACT_TOLERANCE,
    ):
        raise ValueError(
            "MUJOCO_TABLETOP_STATIC_DYNAMIC_FRICTION_UNSUPPORTED:"
            f"static={task_config.object_static_friction}:"
            f"dynamic={task_config.object_dynamic_friction}"
        )

    if not math.isclose(
        float(task_config.object_restitution),
        0.0,
        rel_tol=0.0,
        abs_tol=_CONTRACT_TOLERANCE,
    ):
        raise ValueError(
            "MUJOCO_TABLETOP_NONZERO_RESTITUTION_UNSUPPORTED:"
            f"{task_config.object_restitution}"
        )


def author_tabletop_scene(
    *,
    mujoco: Any,
    spec: Any,
    task_config: TabletopGraspLiftConfig,
) -> MuJoCoTabletopSceneHandles:
    """Author the frozen B2 tabletop scene into an uncompiled/editable MjSpec.

    Task 4 owns only scene structure:
      - static table collision box;
      - dynamic free-joint cube;
      - mocap anchor at the exact initial cube pose;
      - initially-active body weld between cube and anchor.

    Runtime pose publication, kinematic release and contact telemetry are
    intentionally deferred to later B2 tasks.
    """

    _validate_task_contract(task_config)

    for accessor, name in (
        ("geom", _TABLE_GEOM_NAME),
        ("body", _OBJECT_BODY_NAME),
        ("geom", _OBJECT_GEOM_NAME),
        ("joint", _OBJECT_FREE_JOINT_NAME),
        ("body", _OBJECT_ANCHOR_BODY_NAME),
        ("equality", _OBJECT_WELD_NAME),
    ):
        _require_available_name(spec, accessor, name)

    table_half = box_half_extents(task_config.table_dimensions_xyz_m)
    object_half = box_half_extents(task_config.object_dimensions_xyz_m)
    table_quat = yaw_quaternion_wxyz(task_config.table_yaw_rad)
    object_quat = yaw_quaternion_wxyz(task_config.object_yaw_rad)
    slide_friction = float(task_config.object_dynamic_friction)

    spec.worldbody.add_geom(
        name=_TABLE_GEOM_NAME,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=list(task_config.table_center_world_m),
        quat=list(table_quat),
        size=list(table_half),
        contype=_EXTERNAL_CONTYPE,
        conaffinity=_EXTERNAL_CONAFFINITY,
        friction=[
            slide_friction,
            _DEFAULT_ROLLING_FRICTION,
            _DEFAULT_TORSIONAL_FRICTION,
        ],
        rgba=[0.45, 0.45, 0.45, 1.0],
    )

    object_body = spec.worldbody.add_body(
        name=_OBJECT_BODY_NAME,
        pos=list(task_config.object_position_world_m),
        quat=list(object_quat),
    )
    object_body.add_joint(
        name=_OBJECT_FREE_JOINT_NAME,
        type=mujoco.mjtJoint.mjJNT_FREE,
    )
    object_body.add_geom(
        name=_OBJECT_GEOM_NAME,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=list(object_half),
        mass=float(task_config.object_mass_kg),
        contype=_EXTERNAL_CONTYPE,
        conaffinity=_EXTERNAL_CONAFFINITY,
        friction=[
            slide_friction,
            _DEFAULT_ROLLING_FRICTION,
            _DEFAULT_TORSIONAL_FRICTION,
        ],
        rgba=[0.72, 0.45, 0.18, 1.0],
    )

    spec.worldbody.add_body(
        name=_OBJECT_ANCHOR_BODY_NAME,
        pos=list(task_config.object_position_world_m),
        quat=list(object_quat),
        mocap=True,
    )

    spec.add_equality(
        name=_OBJECT_WELD_NAME,
        type=mujoco.mjtEq.mjEQ_WELD,
        objtype=mujoco.mjtObj.mjOBJ_BODY,
        name1=_OBJECT_BODY_NAME,
        name2=_OBJECT_ANCHOR_BODY_NAME,
        active=True,
    )

    return MuJoCoTabletopSceneHandles(
        table_geom_name=_TABLE_GEOM_NAME,
        object_body_name=_OBJECT_BODY_NAME,
        object_geom_name=_OBJECT_GEOM_NAME,
        object_free_joint_name=_OBJECT_FREE_JOINT_NAME,
        object_anchor_body_name=_OBJECT_ANCHOR_BODY_NAME,
        object_weld_name=_OBJECT_WELD_NAME,
        object_initial_position_world_m=tuple(
            float(value) for value in task_config.object_position_world_m
        ),
        object_initial_quaternion_wxyz=object_quat,
    )
