#!/usr/bin/env python3
"""Export deterministic, ROS-independent SG2 whiteboard wipe waypoints.

The output is a geometry contract for a later real-robot integration, not a
robot driver.  In particular, it creates no ROS nodes, publishers, actions, or
hardware commands.  The configured eraser is a *hypothetical fixed end
adapter*; its TCP and any contact-force behaviour must be measured and
validated on the physical robot before use.
"""

from __future__ import annotations

import argparse
import json
from math import isfinite, sqrt
from pathlib import Path
from typing import Any, Mapping, Sequence


EPSILON = 1.0e-6


def generate_paths(config: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Return five EE poses for each configured wipe band.

    The board origin denotes the centre of the robot-facing surface. ``normal``
    points from the board towards the robot, ``u`` is the horizontal wipe axis,
    and ``v`` is the vertical axis.  ``ee_to_contact_rotation_xyzw`` expresses
    the tool-contact frame in the EE frame; it lets an end-mounted eraser face
    the board while the bare EE uses a collision-safe wrist orientation.
    """
    if not isinstance(config, Mapping):
        raise ValueError("config must be a mapping")
    robot = _mapping(config, "robot")
    board = _mapping(config, "board")
    tool = _mapping(config, "tool")
    wipe = _mapping(config, "wipe")

    group = _string(robot, "planning_group")
    ee_frame = _string(robot, "ee_frame")
    origin = _vector(board, "origin", 3)
    normal = _unit_vector(board, "normal")
    u_axis = _unit_vector(board, "u")
    v_axis = _unit_vector(board, "v")
    _validate_frame(normal, u_axis, v_axis)

    ee_to_contact = _vector(tool, "ee_to_contact_local", 3)
    tool_rotation = _unit_quaternion(tool, "ee_to_contact_rotation_xyzw")
    pad_width = _positive(tool, "pad_width")
    pad_height = _positive(tool, "pad_height")
    nominal = _nonnegative(wipe, "nominal_standoff")
    approach = _nonnegative(wipe, "approach_standoff")
    speed = _positive(wipe, "speed")
    if approach < nominal:
        raise ValueError("approach_standoff must be at least nominal_standoff")
    bands = _finite_sequence(wipe, "bands_v")
    if not bands:
        raise ValueError("bands_v must not be empty")
    u_start = _finite_number(wipe.get("u_start"), "u_start")
    u_end = _finite_number(wipe.get("u_end"), "u_end")
    if u_start == u_end:
        raise ValueError("u_start and u_end must differ")
    width = _positive({'value': board.get('width_m', 0.36)}, 'value')
    height = _positive({'value': board.get('height_m', 0.30)}, 'value')
    if max(abs(u_start), abs(u_end)) + pad_width/2 > width/2 + EPSILON:
        raise ValueError('stroke footprint exceeds declared board width')
    if any(abs(v) + pad_height/2 > height/2 + EPSILON for v in bands):
        raise ValueError('stroke footprint exceeds declared board height')
    ordered = sorted(bands)
    if any(b-a > pad_height + EPSILON for a, b in zip(ordered, ordered[1:])):
        raise ValueError('pad height leaves a gap between requested bands')

    # Desired contact-frame columns. Its local +x faces the board, toward -normal.
    contact_rotation = (_neg(normal), _neg(u_axis), v_axis)
    contact_quaternion = _rotation_columns_to_quaternion(contact_rotation)
    # world_R_EE = world_R_contact * inverse(EE_R_contact)
    quaternion = _quaternion_multiply(contact_quaternion, _quaternion_inverse(tool_rotation))
    ee_rotation = _quaternion_to_columns(quaternion)

    result: list[dict[str, Any]] = []
    for index, band_v in enumerate(bands):
        start = _add(origin, _scale(u_axis, u_start), _scale(v_axis, band_v))
        end = _add(origin, _scale(u_axis, u_end), _scale(v_axis, band_v))
        stages = (
            ("approach_start", start, approach),
            ("near_start", start, nominal),
            ("near_end", end, nominal),
            ("return_start", start, nominal),
            ("retreat", start, approach),
        )
        poses = []
        for phase, surface_contact, standoff in stages:
            contact = _add(surface_contact, _scale(normal, standoff))
            ee = _subtract(contact, _rotate(ee_rotation, ee_to_contact))
            poses.append({
                "phase": phase,
                "standoff_m": standoff,
                "contact_xyz": list(contact),
                "ee_xyz": list(ee),
                "quaternion_xyzw": list(quaternion),
                "contact_quaternion_xyzw": list(contact_quaternion),
            })
        result.append({
            "band_index": index,
            "v_offset_m": band_v,
            "planning_group": group,
            "ee_frame": ee_frame,
            "tool_footprint_m": {"width": pad_width, "height": pad_height},
            "suggested_max_contact_speed_m_s": speed,
            "declared_board_size_m": [width, height],
            "coverage_check": "overlapping bands within board; does not prove arbitrary text removal",
            "target_poses": poses,
        })
    return result


def export_document(config: Mapping[str, Any]) -> dict[str, Any]:
    """Build a JSON-serializable transfer document including its limitations."""
    paths = generate_paths(config)
    return {
        "schema": "aiworker-whiteboard-transfer/v1",
        "contract": {
            "adapter": "fixed hypothetical end adapter; physical TCP not yet verified",
            "contact": "geometric standoff path only; no force, impedance, friction, or pigment model",
            "collision_basis": "the identity/+x side-adapter hypothesis was rejected by collision-aware state validity; this end-mounted geometry only avoids that fixed-model collision",
            "execution": "export only; no ROS publishers, actions, or hardware commands",
        },
        "config": config,
        "bands": paths,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        document = export_document(config)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return 0


def _mapping(value: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    result = value.get(name)
    if not isinstance(result, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return result


def _string(value: Mapping[str, Any], name: str) -> str:
    result = value.get(name)
    if not isinstance(result, str) or not result:
        raise ValueError(f"{name} must be a non-empty string")
    return result


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _positive(value: Mapping[str, Any], name: str) -> float:
    result = _finite_number(value.get(name), name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative(value: Mapping[str, Any], name: str) -> float:
    result = _finite_number(value.get(name), name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _finite_sequence(value: Mapping[str, Any], name: str) -> tuple[float, ...]:
    raw = value.get(name)
    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence):
        raise ValueError(f"{name} must be a sequence")
    return tuple(_finite_number(item, name) for item in raw)


def _vector(value: Mapping[str, Any], name: str, length: int) -> tuple[float, ...]:
    result = _finite_sequence(value, name)
    if len(result) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    return result


def _unit_vector(value: Mapping[str, Any], name: str) -> tuple[float, float, float]:
    vector = _vector(value, name, 3)
    norm = _norm(vector)
    if abs(norm - 1.0) > EPSILON:
        raise ValueError(f"{name} must be a unit vector")
    return vector  # type: ignore[return-value]


def _unit_quaternion(value: Mapping[str, Any], name: str) -> tuple[float, float, float, float]:
    quaternion = _vector(value, name, 4)
    norm = _norm(quaternion)
    if norm <= EPSILON:
        raise ValueError(f"{name} must not be zero")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def _validate_frame(normal: Sequence[float], u_axis: Sequence[float], v_axis: Sequence[float]) -> None:
    if any(abs(_dot(first, second)) > EPSILON for first, second in (
        (normal, u_axis), (normal, v_axis), (u_axis, v_axis),
    )):
        raise ValueError("board normal, u, and v must be orthogonal")
    if any(abs(a - b) > EPSILON for a, b in zip(_cross(normal, u_axis), v_axis)):
        raise ValueError("board vectors must satisfy cross(normal, u) == v")


def _norm(vector: Sequence[float]) -> float:
    return sqrt(sum(item * item for item in vector))


def _dot(first: Sequence[float], second: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(first, second))


def _cross(first: Sequence[float], second: Sequence[float]) -> tuple[float, float, float]:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _add(*vectors: Sequence[float]) -> tuple[float, float, float]:
    return tuple(sum(values) for values in zip(*vectors))  # type: ignore[return-value]


def _subtract(first: Sequence[float], second: Sequence[float]) -> tuple[float, float, float]:
    return tuple(a - b for a, b in zip(first, second))  # type: ignore[return-value]


def _scale(vector: Sequence[float], scalar: float) -> tuple[float, float, float]:
    return tuple(value * scalar for value in vector)  # type: ignore[return-value]


def _neg(vector: Sequence[float]) -> tuple[float, float, float]:
    return _scale(vector, -1.0)


def _rotate(columns: Sequence[Sequence[float]], vector: Sequence[float]) -> tuple[float, float, float]:
    return _add(*(_scale(column, component) for column, component in zip(columns, vector)))


def _quaternion_inverse(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quaternion_multiply(first: Sequence[float], second: Sequence[float]) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    result = (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
    norm = _norm(result)
    return tuple(component / norm for component in result)  # type: ignore[return-value]


def _quaternion_to_columns(quaternion: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = quaternion
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)),
        (2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w)),
        (2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _rotation_columns_to_quaternion(columns: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    """Convert a proper 3x3 rotation, given by columns, to normalized XYZW."""
    (r00, r10, r20), (r01, r11, r21), (r02, r12, r22) = columns
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        q = ((r21 - r12) / scale, (r02 - r20) / scale, (r10 - r01) / scale, 0.25 * scale)
    elif r00 > r11 and r00 > r22:
        scale = sqrt(1.0 + r00 - r11 - r22) * 2.0
        q = (0.25 * scale, (r01 + r10) / scale, (r02 + r20) / scale, (r21 - r12) / scale)
    elif r11 > r22:
        scale = sqrt(1.0 + r11 - r00 - r22) * 2.0
        q = ((r01 + r10) / scale, 0.25 * scale, (r12 + r21) / scale, (r02 - r20) / scale)
    else:
        scale = sqrt(1.0 + r22 - r00 - r11) * 2.0
        q = ((r02 + r20) / scale, (r12 + r21) / scale, 0.25 * scale, (r10 - r01) / scale)
    norm = sqrt(sum(component * component for component in q))
    return tuple(component / norm for component in q)  # type: ignore[return-value]


if __name__ == "__main__":
    raise SystemExit(main())
