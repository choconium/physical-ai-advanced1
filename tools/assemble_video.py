#!/usr/bin/env python3
"""Trim actual motion intervals from a real screen recording, never pad or loop."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def probe(path, count_frames=False):
    return json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', *(['-count_frames'] if count_frames else []),
        '-show_format', '-show_streams', '-of', 'json', str(path)]))


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture-dir', required=True, type=Path)
    parser.add_argument('--motion-log', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--target-seconds', default=120., type=float)
    args = parser.parse_args()
    if not 60 <= args.target_seconds <= 180:
        parser.error('target-seconds must be 60..180')
    capture = json.loads((args.capture_dir / 'capture.json').read_text())
    if capture.get('command_exit_code') != 0:
        raise ValueError('mission must finish successfully before making submission video')
    if capture.get('recorder_exit_code') not in (0, 255):
        raise ValueError('screen recorder did not finish successfully')
    raw = args.capture_dir / 'raw.mkv'
    if not raw.is_file():
        raise ValueError('raw recording is missing')
    raw_info = probe(raw, count_frames=True)
    video_stream = next(s for s in raw_info['streams'] if s['codec_type'] == 'video')
    frame_count = int(video_stream['nb_read_frames'])
    fps = capture.get('capture_fps', 24)  # schema1 recordings used fixed24fps
    if fps != 24:
        raise ValueError('only the fixed24fps recorder is supported')
    duration = frame_count / fps
    capture_wall = capture['capture_end_monotonic_s'] - capture['capture_start_monotonic_s']
    if abs(duration - capture_wall) > 1.0:
        raise ValueError('capture frame count does not match monotonic duration; inspect dropped capture frames')
    # The mission logs real monotonic receipt times in the same container/clock.
    records = [json.loads(line) for line in args.motion_log.read_text().splitlines()]
    moving = [r['wall_monotonic_s'] - capture['capture_start_monotonic_s']
              for r in records if r.get('moving') is True]
    intervals = []
    for stamp in moving:
        begin, end = max(0., stamp - .3), min(duration, stamp + .3)
        if end <= begin:
            continue
        if intervals and begin <= intervals[-1][1]:
            intervals[-1][1] = max(end, intervals[-1][1])
        else:
            intervals.append([begin, end])
    active = sum(end - begin for begin, end in intervals)
    if active < 60:
        raise ValueError(f'only {active:.2f}s of motion; record more genuine driving (no padding/slowdown)')
    speed = max(1., active / args.target_seconds)
    if speed > 2:
        raise ValueError('more than 2x required; choose target up to180s or record a shorter real mission')
    # WSL's wall clock can jump. Keep each captured frame exactly once and
    # rebuild its timestamp at the capture FPS before mapping monotonic logs.
    split = ''.join(f'[src{i}]' for i in range(len(intervals)))
    parts = [f'[0:v]setpts=N/({fps}*TB),split={len(intervals)}' + split]
    parts += [f'[src{i}]trim=start={begin:.6f}:end={end:.6f},setpts=PTS-STARTPTS[v{i}]'
              for i, (begin, end) in enumerate(intervals)]
    inputs = ''.join(f'[v{i}]' for i in range(len(intervals)))
    # Drop only the simulator menu and private X11 taskbar, preserving pixels/aspect.
    filt = inputs + f'concat=n={len(intervals)}:v=1:a=0,crop=1280:640:0:50,setpts=PTS/{speed:.9f}'
    if speed > 1.001:
        filt += f",drawtext=text='{speed:.2f}x':x=w-tw-20:y=20:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.5"
    parts.append(filt + '[out]')
    script = args.capture_dir / 'video_filters.txt'
    script.write_text(';\n'.join(parts))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-n', '-i', str(raw),
                    '-filter_complex_script', str(script), '-map', '[out]', '-an', '-r', '24',
                    '-c:v', 'libx264', '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(args.output)], check=True)
    info = probe(args.output)
    final_duration = float(info['format']['duration'])
    if not 60 <= final_duration <= 180:
        raise ValueError(f'output duration violates video requirement: {final_duration}')
    evidence = {'source': 'actual Gazebo screen capture', 'duration_s': final_duration,
                'raw_duration_s': float(raw_info['format']['duration']),
                'normalized_capture_duration_s': duration, 'captured_frames': frame_count,
                'capture_wall_duration_s': capture_wall, 'timestamps_normalized_from_frame_index': True,
                'retained_motion_s': active, 'speed': speed,
                'intervals_s': intervals, 'raw_sha256': digest(raw),
                'motion_log_sha256': digest(args.motion_log), 'video_sha256': digest(args.output),
                'slides': False, 'padded_frames': False, 'looped_frames': False}
    args.output.with_suffix('.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    main()
