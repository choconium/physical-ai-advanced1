"""Seeded tabular Q-learning for choosing the next unvisited wipe band.

This module is intentionally ROS-independent.  It learns on the configured
2-D waypoint graph, then returns a deterministic greedy action at runtime.
The task runner handles MoveIt action results; this module only selects bands.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .core import Mission, load_mission

SCHEMA_VERSION = 1


def mission_sha256(mission: Mission) -> str:
    """Hash the parsed mission contract, independent of JSON whitespace/order."""
    payload = {"initial_pose": asdict(mission.initial_pose), "waypoints": [asdict(item) for item in mission.waypoints],
               "loops": mission.loops, "goal_timeout_s": mission.goal_timeout_s,
               "localization_timeout_s": mission.localization_timeout_s,
               "pose_tolerance_m": mission.pose_tolerance_m, "yaw_tolerance_rad": mission.yaw_tolerance_rad}
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def state_key(current: int, visited_mask: int) -> str:
    return f"{current}:{visited_mask}"


def checkpoint_mask(mission: Mission) -> int:
    return (1 << (len(mission.waypoints) - 1)) - 1


def available_actions(mission: Mission, visited_mask: int) -> tuple[int, ...]:
    all_visited = checkpoint_mask(mission)
    if visited_mask == all_visited:
        return (0,)
    return tuple(index for index in range(1, len(mission.waypoints)) if not visited_mask & (1 << (index - 1)))


def transition(mission: Mission, current: int, visited_mask: int, action: int) -> tuple[int, int, bool]:
    allowed = available_actions(mission, visited_mask)
    if action not in allowed:
        raise ValueError(f"action {action} is not available for state {current}:{visited_mask}")
    if action == 0:
        return 0, visited_mask, True
    return action, visited_mask | (1 << (action - 1)), False


def distance(mission: Mission, current: int, action: int) -> float:
    start, end = mission.waypoints[current].pose, mission.waypoints[action].pose
    return math.hypot(end.x - start.x, end.y - start.y)


def reward(mission: Mission, current: int, visited_mask: int, action: int, done: bool) -> float:
    # Travel cost is always negative. A small reward for visiting checkpoint 1
    # first encodes the stated priority while leaving the other route choices learned.
    value = -distance(mission, current, action)
    if current == 0 and visited_mask == 0 and action == 1:
        value += 1.0
    if done:
        value += 0.5
    return value


def _ensure_row(table: dict[str, dict[str, float]], key: str, actions: tuple[int, ...]) -> dict[str, float]:
    row = table.setdefault(key, {})
    for action in actions:
        row.setdefault(str(action), 0.0)
    return row


def _greedy(row: dict[str, float], actions: tuple[int, ...]) -> int:
    # Equal Q values (e.g. checkpoints placed symmetrically) resolve to the lower
    # waypoint index; the tie-break is deterministic but is not a learned preference.
    return max(actions, key=lambda action: (row[str(action)], -action))


def train(mission: Mission, *, seed: int = 20261030, episodes: int = 4000,
          alpha: float = 0.20, gamma: float = 0.90, epsilon_start: float = 0.30,
          epsilon_end: float = 0.02) -> dict[str, dict[str, float]]:
    if episodes < 1 or not all(math.isfinite(value) for value in (alpha, gamma, epsilon_start, epsilon_end)):
        raise ValueError("training values must be finite and episodes positive")
    if not 0 < alpha <= 1 or not 0 <= gamma <= 1 or not 0 <= epsilon_end <= epsilon_start <= 1:
        raise ValueError("invalid Q-learning hyperparameters")
    generator, table = random.Random(seed), {}
    for episode in range(episodes):
        current, mask, done = 0, 0, False
        epsilon = epsilon_start + (epsilon_end - epsilon_start) * episode / max(episodes - 1, 1)
        while not done:
            actions = available_actions(mission, mask)
            row = _ensure_row(table, state_key(current, mask), actions)
            action = generator.choice(actions) if generator.random() < epsilon else _greedy(row, actions)
            next_current, next_mask, done = transition(mission, current, mask, action)
            target = reward(mission, current, mask, action, done)
            if not done:
                next_actions = available_actions(mission, next_mask)
                next_row = _ensure_row(table, state_key(next_current, next_mask), next_actions)
                target += gamma * max(next_row[str(candidate)] for candidate in next_actions)
            row[str(action)] += alpha * (target - row[str(action)])
            current, mask = next_current, next_mask
    if not all(math.isfinite(value) for row in table.values() for value in row.values()):
        raise ValueError("training produced a non-finite Q value")
    return table


def make_policy(mission: Mission, table: dict[str, dict[str, float]], *, seed: int, episodes: int) -> dict[str, Any]:
    return {"schema_version": SCHEMA_VERSION, "policy_type": "tabular_q_learning_waypoint_scheduler",
            "mission_sha256": mission_sha256(mission), "checkpoint_count": len(mission.waypoints) - 1,
            "state_contract": {"current": "waypoint index", "visited_mask": "bit i-1 means checkpoint i visited", "home_index": 0},
            "training": {"seed": seed, "episodes": episodes, "alpha": 0.20, "gamma": 0.90,
                         "epsilon_start": 0.30, "epsilon_end": 0.02,
                         "reward": "-euclidean_travel_m + 1.0 if first action is checkpoint 1 + 0.5 return-home terminal"},
            "q_table": table}


def validate_policy(model: Any, mission: Mission) -> dict[str, Any]:
    if not isinstance(model, dict) or model.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported policy schema")
    if model.get("policy_type") != "tabular_q_learning_waypoint_scheduler":
        raise ValueError("unexpected policy type")
    if model.get("mission_sha256") != mission_sha256(mission) or model.get("checkpoint_count") != len(mission.waypoints) - 1:
        raise ValueError("policy does not match this mission")
    table = model.get("q_table")
    if not isinstance(table, dict):
        raise ValueError("policy q_table must be an object")
    for key, row in table.items():
        if not isinstance(key, str) or not isinstance(row, dict):
            raise ValueError("policy q_table is malformed")
        for action, value in row.items():
            if not isinstance(action, str) or isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("policy has a non-finite Q value")
    return model


def select_action(model: dict[str, Any], current: int, visited_mask: int, mission: Mission) -> int:
    validate_policy(model, mission)
    if not isinstance(current, int) or not 0 <= current < len(mission.waypoints):
        raise ValueError("current waypoint index is out of range")
    if not isinstance(visited_mask, int) or not 0 <= visited_mask <= checkpoint_mask(mission):
        raise ValueError("visited mask is out of range")
    actions = available_actions(mission, visited_mask)
    row = model["q_table"].get(state_key(current, visited_mask))
    if not isinstance(row, dict) or any(str(action) not in row for action in actions):
        raise ValueError("policy lacks values for this mission state")
    return _greedy(row, actions)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Train or inspect a deterministic Q-learning wipe-band scheduler")
    commands = parser.add_subparsers(dest="command", required=True)
    train_parser = commands.add_parser("train")
    train_parser.add_argument("--mission", type=Path, required=True)
    train_parser.add_argument("--output", type=Path, required=True)
    train_parser.add_argument("--seed", type=int, default=20261030)
    train_parser.add_argument("--episodes", type=int, default=4000)
    args = parser.parse_args(argv)
    mission = load_mission(args.mission)
    table = train(mission, seed=args.seed, episodes=args.episodes)
    policy = make_policy(mission, table, seed=args.seed, episodes=args.episodes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(policy, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "mission_sha256": policy["mission_sha256"], "states": len(table)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
