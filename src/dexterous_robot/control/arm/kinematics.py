from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from dexterous_robot.core import Pose

WAM7_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.5999998824692834, 2.5999998824692834),
    (-2.0000000169966334, 2.0000000169966334),
    (-2.7999999539821165, 2.7999999539821165),
    (-0.8999999727419, 3.0999998867184413),
    (-4.7599998170498425, 1.2400000594071316),
    (-1.5000000127474749, 1.5000000127474749),
    (-3.0000000254949497, 3.0000000254949497),
)

_WAM_CHAIN: tuple[
    tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]], ...
] = (
    ((0.22, 0.14, 0.346), (-math.pi / 2.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.045, -0.55, 0.0), (math.pi / 2.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((-0.045, 0.0, 0.3), (math.pi / 2.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.06), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
)

_DEFAULT_TCP_TRANSLATION_IN_FLANGE_M = (0.0, 0.0, 0.00991000205278397)
_POSITION_TOL_M = 1.0e-4
_ORIENTATION_TOL_RAD = 1.0e-3
_MAX_ITERS = 80
_FD_EPS_RAD = 1.0e-5
_DAMPING = 1.0e-4
_SEED_REGULARIZATION = 1.0e-8
_ORIENTATION_RESIDUAL_WEIGHT = 0.20
_MAX_STEP_RAD = 0.15


def _vector(values: Sequence[float], width: int, error: str) -> np.ndarray:
    try:
        result = np.asarray(tuple(values), dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(error) from exc
    if result.shape != (width,) or not np.isfinite(result).all():
        raise ValueError(error)
    return result


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    c, s = math.cos(angle), math.sin(angle)
    one_minus_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=float,
    )


def _quat_xyzw_to_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    q = _vector(quaternion_xyzw, 4, "WAM7_TARGET_QUATERNION_INVALID")
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-12:
        raise ValueError("WAM7_TARGET_QUATERNION_INVALID")
    x, y, z, w = q / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _matrix_to_quat_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            x = 0.25 * s
            y = (matrix[0, 1] + matrix[1, 0]) / s
            z = (matrix[0, 2] + matrix[2, 0]) / s
            w = (matrix[2, 1] - matrix[1, 2]) / s
        elif index == 1:
            s = math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2.0
            x = (matrix[0, 1] + matrix[1, 0]) / s
            y = 0.25 * s
            z = (matrix[1, 2] + matrix[2, 1]) / s
            w = (matrix[0, 2] - matrix[2, 0]) / s
        else:
            s = math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
            x = (matrix[0, 2] + matrix[2, 0]) / s
            y = (matrix[1, 2] + matrix[2, 1]) / s
            z = 0.25 * s
            w = (matrix[1, 0] - matrix[0, 1]) / s
    q = np.asarray((x, y, z, w), dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return tuple(float(v) for v in q)  # type: ignore[return-value]


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    theta = math.acos(cosine)
    skew = np.asarray(
        (matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]),
        dtype=float,
    )
    if theta < 1.0e-10:
        return 0.5 * skew
    if math.pi - theta < 1.0e-5:
        values, vectors = np.linalg.eig(matrix)
        axis = np.real(vectors[:, int(np.argmin(np.abs(values - 1.0)))])
        axis /= max(float(np.linalg.norm(axis)), 1.0e-12)
        return axis * theta
    return theta / (2.0 * math.sin(theta)) * skew


class Wam7Kinematics:
    """Pure NumPy WAM7 kinematics for the configured manipulator control TCP.

    The M1 default TCP is coincident with the Linker L20 base and preserves the
    frozen successful mount translation.  No simulator or hardware API is used.
    """

    def __init__(
        self,
        *,
        tcp_translation_in_flange_m: Sequence[float] = _DEFAULT_TCP_TRANSLATION_IN_FLANGE_M,
        frame_id: str = "world",
    ) -> None:
        self._tcp_translation = _vector(tcp_translation_in_flange_m, 3, "WAM7_TCP_TRANSLATION_INVALID")
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("WAM7_FRAME_ID_INVALID")
        self._frame_id = frame_id

    def _transform(self, q_rad: Sequence[float]) -> np.ndarray:
        q = _vector(q_rad, 7, "WAM7_Q_INVALID")
        transform = np.eye(4, dtype=float)
        for angle, (xyz, rpy, axis) in zip(q, _WAM_CHAIN, strict=True):
            authored = np.eye(4, dtype=float)
            authored[:3, :3] = _rpy(*rpy)
            authored[:3, 3] = np.asarray(xyz, dtype=float)
            rotation = np.eye(4, dtype=float)
            rotation[:3, :3] = _axis_angle(axis, float(angle))
            transform = transform @ authored @ rotation
        transform = transform.copy()
        transform[:3, 3] += transform[:3, :3] @ self._tcp_translation
        return transform

    def forward(self, q_rad: Sequence[float]) -> Pose:
        transform = self._transform(q_rad)
        return Pose(
            tuple(float(v) for v in transform[:3, 3]),  # type: ignore[arg-type]
            _matrix_to_quat_xyzw(transform[:3, :3]),
            self._frame_id,
        )

    def _pose_residual(self, q_rad: np.ndarray, target_transform: np.ndarray) -> np.ndarray:
        current = self._transform(q_rad)
        position = current[:3, 3] - target_transform[:3, 3]
        orientation = _rotation_vector(target_transform[:3, :3].T @ current[:3, :3])
        return np.concatenate((position, _ORIENTATION_RESIDUAL_WEIGHT * orientation))

    def solve_pose(self, target: Pose, seed_q_rad: Sequence[float]) -> tuple[float, ...]:
        if not isinstance(target, Pose):
            raise ValueError("WAM7_TARGET_POSE_INVALID")
        if target.frame_id != self._frame_id:
            raise ValueError("WAM7_TARGET_FRAME_INVALID")
        seed = _vector(seed_q_rad, 7, "WAM7_Q_INVALID")
        target_transform = np.eye(4, dtype=float)
        target_transform[:3, :3] = _quat_xyzw_to_matrix(target.quaternion_xyzw)
        target_transform[:3, 3] = _vector(target.position_xyz_m, 3, "WAM7_TARGET_POSITION_INVALID")

        limits = np.asarray(WAM7_JOINT_LIMITS_RAD, dtype=float)
        margin = 1.0e-5
        q = np.clip(seed, limits[:, 0] + margin, limits[:, 1] - margin)
        converged = False
        for iteration in range(_MAX_ITERS + 1):
            residual = self._pose_residual(q, target_transform)
            position_error = float(np.linalg.norm(residual[:3]))
            orientation_error = float(np.linalg.norm(residual[3:]) / _ORIENTATION_RESIDUAL_WEIGHT)
            if position_error <= _POSITION_TOL_M and orientation_error <= _ORIENTATION_TOL_RAD:
                converged = True
                break
            if iteration >= _MAX_ITERS:
                break
            jacobian = np.zeros((6, 7), dtype=float)
            for joint_index in range(7):
                plus = q.copy(); plus[joint_index] += _FD_EPS_RAD
                minus = q.copy(); minus[joint_index] -= _FD_EPS_RAD
                jacobian[:, joint_index] = (
                    self._pose_residual(plus, target_transform) - self._pose_residual(minus, target_transform)
                ) / (2.0 * _FD_EPS_RAD)
            hessian = jacobian.T @ jacobian + (_DAMPING**2 + _SEED_REGULARIZATION) * np.eye(7)
            gradient = jacobian.T @ residual + _SEED_REGULARIZATION * (q - seed)
            delta = -np.linalg.solve(hessian, gradient)
            peak = float(np.max(np.abs(delta)))
            if peak > _MAX_STEP_RAD:
                delta *= _MAX_STEP_RAD / peak
            q = np.clip(q + delta, limits[:, 0] + margin, limits[:, 1] - margin)

        if not converged:
            raise RuntimeError("WAM7_IK_DID_NOT_CONVERGE")
        return tuple(float(v) for v in q)
