from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import sqrt
from typing import Mapping, Sequence

SESSION_DYNAMIC_PROPERTIES_TO_RELEASE = ("xformOp:translate", "xformOp:orient")
SESSION_PROPERTIES_TO_PRESERVE = ("xformOp:scale", "xformOpOrder")


def _xyz(values: Sequence[float] | None) -> tuple[float, float, float] | None:
    if values is None:
        return None
    if len(values) != 3:
        raise ValueError("TRANSFORM_POSITION_WIDTH_INVALID")
    return (float(values[0]), float(values[1]), float(values[2]))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))


@dataclass(frozen=True)
class PositionSourceAudit:
    positions: Mapping[str, tuple[float, float, float] | None]
    max_pairwise_position_error_m: float
    consistent: bool


def compare_position_sources(
    *,
    tensor_xyz: Sequence[float] | None,
    physx_xyz: Sequence[float] | None,
    usd_xyz: Sequence[float] | None,
    fabric_xyz: Sequence[float] | None,
    tolerance_m: float,
) -> PositionSourceAudit:
    tolerance = float(tolerance_m)
    if tolerance <= 0.0:
        raise ValueError("TRANSFORM_TOLERANCE_INVALID")
    positions = {
        "tensor": _xyz(tensor_xyz),
        "physx": _xyz(physx_xyz),
        "usd": _xyz(usd_xyz),
        "fabric": _xyz(fabric_xyz),
    }
    available = [(name, value) for name, value in positions.items() if value is not None]
    if len(available) < 3:
        max_error = float("inf")
        consistent = False
    else:
        errors = [_distance(a, b) for (_, a), (_, b) in combinations(available, 2)]
        max_error = max(errors, default=0.0)
        consistent = max_error <= tolerance
    return PositionSourceAudit(positions=positions, max_pairwise_position_error_m=max_error, consistent=consistent)


class RootSeedDynamicTransformPolicy:
    """Runtime-only dynamic transform policy.

    Simulator-specific modules are imported inside methods so importing this
    package under normal Python never initializes Kit/PhysX/Fabric.
    """

    def __init__(self, *, prim_path: str, update_to_fast_cache: bool, update_to_usd: bool, tolerance_m: float) -> None:
        if not isinstance(prim_path, str) or not prim_path.startswith("/"):
            raise ValueError("TRANSFORM_SYNC_PRIM_PATH_INVALID")
        self.prim_path = prim_path
        self.update_to_fast_cache = bool(update_to_fast_cache)
        self.update_to_usd = bool(update_to_usd)
        self.tolerance_m = float(tolerance_m)
        if self.tolerance_m <= 0.0:
            raise ValueError("TRANSFORM_SYNC_TOLERANCE_INVALID")

    def sync(self, physx) -> None:
        physx.update_transformations(self.update_to_fast_cache, self.update_to_usd)

    def direct_physx_pose(self, physx) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        raw = physx.get_rigidbody_transformation(self.prim_path)
        if not raw or ("ret_val" in raw and not bool(raw.get("ret_val"))):
            raise RuntimeError("TRANSFORM_SYNC_PHYSX_POSE_UNAVAILABLE")
        position = raw.get("position")
        rotation = raw.get("rotation", raw.get("orientation"))
        if position is None or rotation is None:
            raise RuntimeError("TRANSFORM_SYNC_PHYSX_POSE_UNAVAILABLE")
        return tuple(float(x) for x in position), tuple(float(x) for x in rotation)

    def seed_root_then_release_session(self, *, stage, session_layer, physx, app=None, headless: bool = True) -> dict[str, object]:
        from pxr import Gf, Sdf, Usd, UsdGeom

        position, quat_xyzw = self.direct_physx_pose(physx)
        root_layer = stage.GetRootLayer()
        original_target = stage.GetEditTarget()
        stage.SetEditTarget(stage.GetEditTargetForLocalLayer(root_layer))
        try:
            prim = stage.GetPrimAtPath(self.prim_path)
            prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*position))
            x, y, z, w = quat_xyzw
            prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
        finally:
            stage.SetEditTarget(original_target)

        prim_spec = session_layer.GetPrimAtPath(Sdf.Path(self.prim_path))
        if prim_spec is None:
            raise RuntimeError("TRANSFORM_SYNC_SESSION_PRIM_SPEC_MISSING")
        before = {str(prop.name) for prop in prim_spec.properties}
        for name in SESSION_DYNAMIC_PROPERTIES_TO_RELEASE:
            prop_spec = session_layer.GetPropertyAtPath(Sdf.Path(f"{self.prim_path}.{name}"))
            if prop_spec is None:
                raise RuntimeError(f"TRANSFORM_SYNC_SESSION_PROPERTY_MISSING:{name}")
            prim_spec.RemoveProperty(prop_spec)
        after_spec = session_layer.GetPrimAtPath(Sdf.Path(self.prim_path))
        after = {str(prop.name) for prop in after_spec.properties} if after_spec is not None else set()
        if any(name in after for name in SESSION_DYNAMIC_PROPERTIES_TO_RELEASE):
            raise RuntimeError("TRANSFORM_SYNC_SESSION_RELEASE_FAILED")
        if any(name not in after for name in SESSION_PROPERTIES_TO_PRESERVE):
            raise RuntimeError("TRANSFORM_SYNC_PRESERVED_PROPERTY_LOST")

        self.sync(physx)
        if app is not None and not headless:
            app.update()
        physx_position, _ = self.direct_physx_pose(physx)
        prim = stage.GetPrimAtPath(self.prim_path)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        usd_position = tuple(float(x) for x in matrix.ExtractTranslation())
        error = _distance(_xyz(physx_position), _xyz(usd_position))  # type: ignore[arg-type]
        if error > self.tolerance_m:
            # Restore a strong session pose so a display repair never leaves the
            # composed stage at an invalid pose. The caller still sees failure.
            stage.SetEditTarget(stage.GetEditTargetForLocalLayer(session_layer))
            try:
                prim = stage.GetPrimAtPath(self.prim_path)
                prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*physx_position))
                x, y, z, w = quat_xyzw
                prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
            finally:
                stage.SetEditTarget(original_target)
            self.sync(physx)
            raise RuntimeError(f"TRANSFORM_SYNC_INITIAL_COMPOSED_MISMATCH:{error}")

        return {
            "prim_path": self.prim_path,
            "root_layer_identifier": str(root_layer.identifier),
            "released_properties": list(SESSION_DYNAMIC_PROPERTIES_TO_RELEASE),
            "preserved_properties": list(SESSION_PROPERTIES_TO_PRESERVE),
            "properties_before": sorted(before),
            "properties_after": sorted(after),
            "physx_position_world_m": list(physx_position),
            "composed_usd_position_world_m": list(usd_position),
            "position_error_m": error,
        }
