#!/usr/bin/env python3
"""Set the Gazebo GUI camera to a reproducible whiteboard-wipe view.

Only ``/gui/move_to/pose`` is called.  It does not command the robot, modify
physics, or change any model pose.  Gazebo's GUI camera looks along its local
``+X`` axis; :func:`look_at_quaternion` therefore builds a right-handed camera
frame whose local ``+X`` points from the camera to the requested target.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Sequence

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.transport13 import Node


# Robot-facing view: the board front lies at x=0.52 and faces negative x.
# It includes the arm, the full 0.36 x 0.30 m board, and its AI bitmap.
DEFAULT_CAMERA = (0.15, -0.55, 1.20)
DEFAULT_TARGET = (0.49, -0.28, 1.05)
SERVICE = "/gui/move_to/pose"


def look_at_quaternion(
    camera_xyz: Sequence[float], target_xyz: Sequence[float]
) -> tuple[float, float, float, float]:
    """Return xyzw for a Gazebo camera whose local +X looks at ``target_xyz``."""

    camera = _triple(camera_xyz, "camera")
    target = _triple(target_xyz, "target")
    forward = _unit(tuple(target[index] - camera[index] for index in range(3)))
    world_up = (0.0, 0.0, 1.0)
    # Camera local Z is the world-up projection into the view plane.  This
    # avoids a roll while retaining a proper right-handed (+X,+Y,+Z) frame.
    up = _unit(
        tuple(
            world_up[index] - _dot(world_up, forward) * forward[index]
            for index in range(3)
        )
    )
    right = _cross(up, forward)  # forward x right = up
    return _matrix_columns_to_xyzw(forward, right, up)


def configure_view(
    camera_xyz: Sequence[float] = DEFAULT_CAMERA,
    target_xyz: Sequence[float] = DEFAULT_TARGET,
    timeout_ms: int = 3000,
) -> bool:
    """Request the camera pose and return Gazebo's service acknowledgement."""

    camera = _triple(camera_xyz, "camera")
    quaternion = look_at_quaternion(camera, target_xyz)
    request = GUICamera()
    request.pose.position.x, request.pose.position.y, request.pose.position.z = camera
    (
        request.pose.orientation.x,
        request.pose.orientation.y,
        request.pose.orientation.z,
        request.pose.orientation.w,
    ) = quaternion
    executed, response = Node().request(
        SERVICE, request, GUICamera, Boolean, timeout_ms
    )
    return bool(executed and response.data)


def _triple(values: Sequence[float], label: str) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)) or len(values) != 3:
        raise ValueError(f"{label} must contain three finite coordinates")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain three finite coordinates")
    return result  # type: ignore[return-value]


def _unit(vector: Sequence[float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(value * value for value in vector))
    if length < 1e-9:
        raise ValueError("camera and target must differ and not be vertically aligned")
    return tuple(value / length for value in vector)  # type: ignore[return-value]


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def _cross(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _matrix_columns_to_xyzw(
    column_x: Sequence[float], column_y: Sequence[float], column_z: Sequence[float]
) -> tuple[float, float, float, float]:
    """Convert rotation matrix columns to a normalized xyzw quaternion."""

    m00, m10, m20 = column_x
    m01, m11, m21 = column_y
    m02, m12, m22 = column_z
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale)
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale)
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale)
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", nargs=3, type=float, default=DEFAULT_CAMERA, metavar=("X", "Y", "Z"))
    parser.add_argument("--target", nargs=3, type=float, default=DEFAULT_TARGET, metavar=("X", "Y", "Z"))
    parser.add_argument("--timeout-ms", type=int, default=3000)
    args = parser.parse_args()
    try:
        ok = configure_view(args.camera, args.target, args.timeout_ms)
    except (RuntimeError, ValueError) as error:
        print(f"camera configuration failed: {error}", file=sys.stderr)
        return 2
    if not ok:
        print(f"Gazebo did not acknowledge {SERVICE}", file=sys.stderr)
        return 1
    quat = look_at_quaternion(args.camera, args.target)
    print("camera configured", "camera=", tuple(args.camera), "target=", tuple(args.target), "xyzw=", quat)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
