#!/usr/bin/env python3
"""AI-selected bands, actual MoveIt execution and TF-driven surrogate erasure."""
import argparse
from dataclasses import asdict, replace
import json
import hashlib
import math
from pathlib import Path
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from PIL import Image, ImageDraw
from moveit_adapter import SimMoveIt, ee_pose as pose
from ink_model import initial_pixels, BoardConfig, apply_tool_sample, coverage_metrics, build_sdf
from gazebo_scene import GazeboScene
from export_transfer import generate_paths, export_document
from gazebo_aiworker_patrol.core import Mission, Pose2D, Waypoint
from gazebo_aiworker_patrol.q_scheduler import train, make_policy, select_action, transition


BANDS = (1.0875, 1.05, 1.0125)


def mask(pixels, path):
    image = Image.new('L', (360, 300), 0)
    draw = ImageDraw.Draw(image)
    for p in pixels:
        if not p.erased:
            u, v = (p.y + .46) * 1000, (1.2 - p.z) * 1000
            draw.rectangle([round(u-7), round(v-7), round(u+6), round(v+6)], fill=255)
    image.save(path)


def stretch_trajectory(trajectory, minimum_s):
    points = trajectory.joint_trajectory.points
    duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
    if duration <= 0:
        raise ValueError('trajectory duration must be positive')
    scale = max(1., minimum_s / duration)
    for p in points:
        total_ns = round((p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) * scale * 1e9)
        p.time_from_start.sec, p.time_from_start.nanosec = divmod(total_ns, 1_000_000_000)
        p.velocities = [v / scale for v in p.velocities]
        p.accelerations = [a / (scale * scale) for a in p.accelerations]
    return trajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--execute-sim', action='store_true')
    parser.add_argument('--reuse-scene', action='store_true', help='only for a previously spawned, uncleared board')
    parser.add_argument('--config', type=Path, default=Path('config/transfer.example.json'))
    args = parser.parse_args()
    if not args.execute_sim:
        parser.error('this test requires explicit --execute-sim; no real-robot mode exists')
    if not args.run_id.replace('_', '').replace('-', '').isalnum() or len(args.run_id) > 64:
        parser.error('run-id must be identifier-safe')
    config = json.loads(args.config.read_text())
    profile_hash = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if profile_hash != '7879b76f1ac4eb2cf3a6a1d71db2ba4e095af934178ba2d630eb67f666086475':
        parser.error('execution supports only the tested simulation profile; export other geometry for review')
    paths = generate_paths(config)
    run = Path('artifacts') / args.run_id
    run.mkdir(parents=True, exist_ok=False)
    board, pixels = BoardConfig(), initial_pixels()
    mask(pixels, run / 'before_mask.png')
    (run / 'board.sdf').write_text(build_sdf(board, pixels))
    (run / 'transfer_paths.json').write_text(json.dumps(export_document(config), indent=2))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node, scene = SimMoveIt(), GazeboScene()
    if not args.reuse_scene:
        scene.spawn_board(build_sdf(board, pixels))
    events = (run / 'events.jsonl').open('x')
    samples = (run / 'tool_samples.jsonl').open('x')
    start = time.monotonic()
    enabled, last_visual, sample_count = False, 0., 0
    completed, ok, reason = 0, False, ''
    tf_rejected, last_tf_error = 0, ''

    def event(name, **fields):
        events.write(json.dumps({'event': name, 'wall_monotonic_s': time.monotonic(),
            'elapsed_wall_s': time.monotonic()-start, **fields}, allow_nan=False) + '\n')
        events.flush()

    def sample():
        nonlocal pixels, last_visual, sample_count, tf_rejected, last_tf_error
        try:
            xyz, q = node.measured_tip()
        except Exception as exc:
            tf_rejected += 1
            last_tf_error = str(exc)
            if tf_rejected == 1 or tf_rejected % 50 == 0:
                event('tf_rejected', count=tf_rejected, reason=last_tf_error)
            return  # stale/missing TF cannot erase any cell
        # Contact frame = EE * Ry(+pi/2); its+X is the gripper's local -Z.
        x, y, z, w = q
        s = math.sqrt(.5)
        contact_q = [s*(x-z), s*(w+y), s*(z+x), s*(w-y)]
        x, y, z, w = contact_q
        axis = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)]
        tip = [v + .08*d for v, d in zip(xyz, axis)]
        updated = apply_tool_sample(pixels, tip, contact_q, enabled, board)
        removed = [p.id for p, new in zip(pixels, updated) if new.erased and not p.erased]
        for name in removed:
            scene.erase(name)
            # Commit each cell only after Gazebo confirms that exact deletion.
            pixels = [replace(p, erased=True) if p.id == name else p for p in pixels]
        now = time.monotonic()
        if now - last_visual > .10:
            scene.set_tool([v + .06*d for v, d in zip(xyz, axis)], q)
            last_visual = now
        samples.write(json.dumps({'wall_monotonic_s': now, 'sim_time_s': node.get_clock().now().nanoseconds*1e-9,
            'ee_xyz': xyz, 'contact_tip_xyz': tip, 'ee_q_xyzw': q, 'contact_q_xyzw': contact_q, 'wipe_enabled': enabled,
            'removed': removed, 'residual': coverage_metrics(pixels)['residual_pixels']}) + '\n')
        samples.flush()
        sample_count += 1

    node.sample_hook = sample
    try:
        node.setup_scene()
        mission = Mission(Pose2D(-.14, 1.05, 0.), tuple([
            Waypoint('home', Pose2D(-.14, 1.05, 0.))] +
            [Waypoint('band_' + str(i), Pose2D(-.28, z, 0.)) for i, z in enumerate(BANDS)]),
            1, 120., 30., .01, .2)
        policy = make_policy(mission, train(mission, seed=20261030, episodes=4000), seed=20261030, episodes=4000)
        (run / 'policy.json').write_text(json.dumps(policy, indent=2))
        event('model_definition', surrogate='geometric ink cells, not contact force or real pigment',
              board=asdict(board), ee_to_contact_local=[0,0,-.08], nominal_standoff_m=.005)
        current, visited = 0, 0
        for step in range(3):
            action = select_action(policy, current, visited, mission)
            height = BANDS[action-1]
            event('policy_decision', state=[current, visited], action=action, band_z=height,
                  dirty_cells=coverage_metrics(pixels)['residual_pixels'])
            # Reposition off the board with joint-space planning, allowing the
            # redundant7DOF arm to change configuration between height bands.
            selected_path = paths[action-1]['target_poses']
            waypoints = []
            for target in selected_path:
                value = pose(*target['ee_xyz'])
                value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w = target['quaternion_xyzw']
                waypoints.append(value)
            trajectory = node.plan_approach(waypoints[0])
            node.execute(trajectory)
            event('approach_executed', points=len(trajectory.joint_trajectory.points))
            # A single TOTG path cannot time-parameterize an exact180deg cusp.
            # Plan/stop/execute each straight segment separately at the reversal.
            for segment, (target, minimum_s) in enumerate(zip(waypoints, (1., 2., 13., 13., 2.))):
                trajectory, fraction = node.cartesian([target])
                trajectory = stretch_trajectory(trajectory, minimum_s)
                event('cartesian_planned', segment=segment, fraction=fraction,
                      points=len(trajectory.joint_trajectory.points))
                enabled = True
                node.execute(trajectory)
                enabled = False
            completed += 1
            current, visited, _ = transition(mission, current, visited, action)
            event('band_executed', band=action, **coverage_metrics(pixels))
        result = coverage_metrics(pixels)
        ok = result['residual_pixels'] == 0 and completed == 3
        reason = 'all planned bands executed and all surrogate ink removed' if ok else 'residual ink remains'
        event('finished', success=ok, **result)
    except (Exception, KeyboardInterrupt) as exc:
        reason = type(exc).__name__ + ': ' + str(exc)
        event('failed', reason=reason)
    finally:
        enabled = False
        node.sample_hook = lambda: None
        mask(pixels, run / 'after_mask.png')
        summary = {'run_id': args.run_id, 'success': ok, 'completed_bands': completed,
                   'reason': reason, 'wall_duration_s': time.monotonic()-start, 'measured_tool_samples': sample_count,
                   'tf_rejected_samples': tf_rejected, 'last_tf_error': last_tf_error,
                   'model': 'TF-driven geometric erasure surrogate; force and pigment physics not simulated', **coverage_metrics(pixels)}
        (run / 'summary.json').write_text(json.dumps(summary, indent=2))
        events.close()
        samples.close()
        node.destroy_node()
        rclpy.try_shutdown()
        print(json.dumps(summary, indent=2), flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
