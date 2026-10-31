"""Small, deterministic whiteboard-ink surrogate for the Gazebo exercise.

This module deliberately models *visible ink cells*, not marker pigment, contact
forces, friction, or an eraser's material deformation. A caller obtains the
contact-tip pose from TF using the fixture transform in transfer.example.json,
feeds samples to :func:`apply_tool_sample`, and can delete
the corresponding independent Gazebo models itself.

At the intended 20 Hz and at most0.02 m/s wipe speed a tool travels1 mm per sample,
which is well below the 20 mm character-cell pitch.  Consequently this compact
model evaluates individual samples and intentionally does not interpolate
between them.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import cos, isfinite, sqrt
from typing import Iterable, Sequence
from xml.sax.saxutils import escape


ERASER_CONTACT_OFFSET_M = 0.08
"""Length of the hypothetical EE-local-Z contact offset; no physical calibration."""

ERASER_WIDTH_Y = 0.07
ERASER_HEIGHT_Z = 0.045
ERASER_ALIGNMENT_DEGREES = 10.0
ERASER_SURFACE_TOLERANCE = 0.008


@dataclass(frozen=True)
class BoardConfig:
    """World-frame geometry for the vertical board's robot-facing surface."""

    surface_x: float = 0.52
    center_y: float = -0.28
    center_z: float = 1.05
    width_y: float = 0.36
    height_z: float = 0.30
    thickness_x: float = 0.02
    ink_x: float = 0.518

    def __post_init__(self) -> None:
        values = (
            self.surface_x,
            self.center_y,
            self.center_z,
            self.width_y,
            self.height_z,
            self.thickness_x,
            self.ink_x,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("board dimensions and pose must be finite")
        if self.width_y <= 0 or self.height_z <= 0 or self.thickness_x <= 0:
            raise ValueError("board dimensions must be positive")

    @property
    def center_x(self) -> float:
        """Centre of the board solid, behind its robot-facing surface."""

        return self.surface_x + self.thickness_x / 2.0

    def contains_yz(self, y: float, z: float) -> bool:
        return (
            abs(y - self.center_y) <= self.width_y / 2.0
            and abs(z - self.center_z) <= self.height_z / 2.0
        )


DEFAULT_BOARD = BoardConfig()


@dataclass(frozen=True)
class InkPixel:
    """One independently visible square of the geometric ink surrogate."""

    id: str
    y: float
    z: float
    erased: bool = False

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("ink pixel id must not be empty")
        if not isfinite(self.y) or not isfinite(self.z):
            raise ValueError("ink pixel coordinates must be finite")


def initial_pixels() -> list[InkPixel]:
    """Return 27 black bitmap cells spelling ``AI`` on the board front.

    The 5-by-5 glyphs occupy y=-0.38..-0.18 m and z=1.00..1.10 m, safely
    within the board bounds.  A new list is created for every call.
    """

    glyphs = (
        (".XXX.", "X...X", "XXXXX", "X...X", "X...X"),  # A: 14 cells
        ("XXXXX", "..X..", "..X..", "..X..", "XXXXX"),  # I: 13 cells
    )
    pixels: list[InkPixel] = []
    identifier = 0
    for glyph_index, glyph in enumerate(glyphs):
        column_offset = glyph_index * 6
        for row, line in enumerate(glyph):
            for column, mark in enumerate(line):
                if mark != "X":
                    continue
                pixels.append(
                    InkPixel(
                        id=f"ink_{identifier:03d}",
                        y=-0.38 + 0.02 * (column_offset + column),
                        z=1.10 - 0.025 * row,
                    )
                )
                identifier += 1
    return pixels


def apply_tool_sample(
    pixels: Iterable[InkPixel],
    tool_xyz: Sequence[float],
    quaternion_xyzw: Sequence[float],
    enabled_wipe: bool,
    board: BoardConfig = DEFAULT_BOARD,
) -> list[InkPixel]:
    """Apply one calibrated eraser contact-tip sample and return new pixels.

    ``tool_xyz`` is the contact tip in world coordinates, *not* the raw
    ``end_effector_r_link`` pose.  The eraser local ``+x`` must point toward
    world ``+x`` within ten degrees and the tip must be within 8 mm of the
    board surface.  Only non-erased pixels inside the 70 by 45 mm footprint
    can be erased.  Input pixels are never mutated.
    """

    pixel_list = list(pixels)
    _validate_pixels(pixel_list, board)
    if not isinstance(enabled_wipe, bool):
        raise ValueError("enabled_wipe must be a bool")
    tip_x, tip_y, tip_z = _finite_vector(tool_xyz, 3, "tool_xyz")
    orientation_x = _local_x_axis(quaternion_xyzw)

    if not enabled_wipe:
        return pixel_list
    if not board.surface_x - ERASER_SURFACE_TOLERANCE <= tip_x <= board.surface_x + 1e-6:
        return pixel_list
    if not board.contains_yz(tip_y, tip_z):
        return pixel_list
    if orientation_x[0] < cos(_radians(ERASER_ALIGNMENT_DEGREES)):
        return pixel_list

    half_y = ERASER_WIDTH_Y / 2.0
    half_z = ERASER_HEIGHT_Z / 2.0
    return [
        replace(pixel, erased=True)
        if (
            not pixel.erased
            and abs(pixel.y - tip_y) <= half_y
            and abs(pixel.z - tip_z) <= half_z
        )
        else pixel
        for pixel in pixel_list
    ]


def coverage_metrics(pixels: Iterable[InkPixel]) -> dict[str, float | int]:
    """Return deterministic coverage counts for a report or controller log."""

    pixel_list = list(pixels)
    _validate_pixels(pixel_list, DEFAULT_BOARD)
    total = len(pixel_list)
    erased = sum(pixel.erased for pixel in pixel_list)
    return {
        "total_pixels": total,
        "erased_pixels": erased,
        "residual_pixels": total - erased,
        "erasure_ratio": 0.0 if total == 0 else erased / total,
    }


def build_sdf(board: BoardConfig, pixels: Iterable[InkPixel]) -> str:
    """Build a self-contained SDF world with a collidable board and ink models.

    Each non-erased ink cell is its own static visual-only ``ink_*`` model.
    This makes visible removal possible through Gazebo's entity-delete service;
    this function intentionally creates no plugins, timers, or deletion logic.
    """

    pixel_list = list(pixels)
    _validate_pixels(pixel_list, board)
    visible = [pixel for pixel in pixel_list if not pixel.erased]
    if len({pixel.id for pixel in visible}) != len(visible):
        raise ValueError("ink pixel ids must be unique")
    board_model = f"""    <model name=\"whiteboard\">
      <static>true</static>
      <pose>{board.center_x:.6f} {board.center_y:.6f} {board.center_z:.6f} 0 0 0</pose>
      <link name=\"board_link\">
        <collision name=\"board_collision\"><geometry><box><size>{board.thickness_x:.6f} {board.width_y:.6f} {board.height_z:.6f}</size></box></geometry></collision>
        <visual name=\"board_visual\"><geometry><box><size>{board.thickness_x:.6f} {board.width_y:.6f} {board.height_z:.6f}</size></box></geometry><material><ambient>0.92 0.92 0.90 1</ambient><diffuse>0.92 0.92 0.90 1</diffuse><emissive>0.8 0.8 0.8 1</emissive></material></visual>
      </link>
    </model>"""
    ink_models = "\n".join(_ink_model_sdf(pixel, board) for pixel in visible)
    return "\n".join(
        (
            '<?xml version="1.0"?>',
            '<sdf version="1.10">',
            '  <world name="whiteboard_ink_world">',
            board_model,
            ink_models,
            '  </world>',
            '</sdf>',
            '',
        )
    )


def _ink_model_sdf(pixel: InkPixel, board: BoardConfig) -> str:
    name = escape(pixel.id, {"\"": "&quot;"})
    return f"""    <model name=\"{name}\">
      <static>true</static>
      <pose>{board.ink_x:.6f} {pixel.y:.6f} {pixel.z:.6f} 0 0 0</pose>
      <link name=\"ink_link\">
        <visual name=\"ink_visual\"><geometry><box><size>0.003 0.014 0.014</size></box></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual>
      </link>
    </model>"""


def _validate_pixels(pixels: Sequence[InkPixel], board: BoardConfig) -> None:
    ids: set[str] = set()
    for pixel in pixels:
        if not isinstance(pixel, InkPixel):
            raise ValueError("pixels must contain InkPixel records")
        if pixel.id in ids:
            raise ValueError("ink pixel ids must be unique")
        ids.add(pixel.id)
        if not board.contains_yz(pixel.y, pixel.z):
            raise ValueError(f"ink pixel {pixel.id!r} is outside board bounds")


def _finite_vector(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)) or len(values) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    result = tuple(float(value) for value in values)
    if not all(isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def _local_x_axis(quaternion_xyzw: Sequence[float]) -> tuple[float, float, float]:
    x, y, z, w = _finite_vector(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("quaternion_xyzw must not be zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # First column of the normalized quaternion's rotation matrix: R * (1,0,0).
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )


def _radians(value: float) -> float:
    # Avoid a broad math import while making the degree-based API explicit.
    return value * 3.141592653589793 / 180.0
