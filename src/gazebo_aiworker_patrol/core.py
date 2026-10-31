"""ROS-independent mission validation and append-only evidence recording."""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    pose: Pose2D


@dataclass(frozen=True)
class Mission:
    initial_pose: Pose2D
    waypoints: tuple[Waypoint, ...]
    loops: int
    goal_timeout_s: float
    localization_timeout_s: float
    pose_tolerance_m: float
    yaw_tolerance_rad: float


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def _pose(value: Any, label: str) -> Pose2D:
    if not isinstance(value, dict) or set(value) != {"x", "y", "yaw"}:
        raise ValueError(f"{label} must have exactly x, y, yaw")
    return Pose2D(*(_number(value[key], f"{label}.{key}") for key in ("x", "y", "yaw")))


def load_mission(path: Path) -> Mission:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path} is not valid JSON") from exc
    fields = {"schema_version", "initial_pose", "waypoints", "loops", "goal_timeout_s", "localization_timeout_s", "pose_tolerance_m", "yaw_tolerance_rad"}
    if not isinstance(raw, dict) or set(raw) != fields or raw["schema_version"] != 1:
        raise ValueError("mission must be schema_version 1 with no unexpected fields")
    if not isinstance(raw["waypoints"], list) or len(raw["waypoints"]) < 2:
        raise ValueError("waypoints must include home and at least one checkpoint")
    seen, waypoints = set(), []
    for index, entry in enumerate(raw["waypoints"]):
        if not isinstance(entry, dict) or set(entry) != {"id", "pose"}:
            raise ValueError(f"waypoints[{index}] must have exactly id and pose")
        waypoint_id = entry["id"]
        if not isinstance(waypoint_id, str) or not IDENTIFIER.fullmatch(waypoint_id) or waypoint_id in seen:
            raise ValueError(f"waypoints[{index}].id must be unique and identifier-safe")
        seen.add(waypoint_id)
        waypoints.append(Waypoint(waypoint_id, _pose(entry["pose"], f"waypoints[{index}].pose")))
    loops = raw["loops"]
    if isinstance(loops, bool) or not isinstance(loops, int) or not 1 <= loops <= 20:
        raise ValueError("loops must be an integer from 1 to 20")
    values = {name: _number(raw[name], name) for name in ("goal_timeout_s", "localization_timeout_s", "pose_tolerance_m", "yaw_tolerance_rad")}
    if values["goal_timeout_s"] <= 0 or values["localization_timeout_s"] <= 0:
        raise ValueError("timeouts must be positive")
    if not 0 < values["pose_tolerance_m"] <= 2 or not 0 < values["yaw_tolerance_rad"] <= math.pi:
        raise ValueError("pose tolerances are outside a safe simulation range")
    return Mission(_pose(raw["initial_pose"], "initial_pose"), tuple(waypoints), loops, **values)


def angle_error(actual: float, target: float) -> float:
    return abs((actual - target + math.pi) % (2 * math.pi) - math.pi)


def pose_reached(actual: Pose2D | None, target: Pose2D, distance_tolerance: float, yaw_tolerance: float) -> bool:
    return actual is not None and math.hypot(actual.x - target.x, actual.y - target.y) <= distance_tolerance and angle_error(actual.yaw, target.yaw) <= yaw_tolerance


class RunRecorder:
    def __init__(self, root: Path, run_id: str) -> None:
        if not IDENTIFIER.fullmatch(run_id):
            raise ValueError("run_id must be a 1-64 character safe identifier")
        root, self.run_id = root.resolve(), run_id
        self.run_directory = (root / run_id).resolve()
        try:
            self.run_directory.relative_to(root)
        except ValueError as exc:
            raise ValueError("run directory must stay under artifact root") from exc
        self.run_directory.mkdir(parents=True, exist_ok=False)
        self.events = (self.run_directory / "events.jsonl").open("x", encoding="utf-8")
        self.odometry_stream = (self.run_directory / "odometry.jsonl").open("x", encoding="utf-8")
        self.started = time.monotonic()

    def _write(self, stream, fields: dict) -> None:
        now = time.monotonic()
        stream.write(json.dumps({"wall_monotonic_s": round(now, 6), "elapsed_wall_s": round(now - self.started, 6), **fields}, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
        stream.flush()

    def event(self, event: str, **fields: Any) -> None:
        self._write(self.events, {"event": event, **fields})

    def odometry(self, **fields: Any) -> None:
        self._write(self.odometry_stream, fields)

    def close(self, summary: dict) -> Path:
        self.events.close()
        self.odometry_stream.close()
        output = self.run_directory / "summary.json"
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        return output
