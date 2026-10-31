#!/usr/bin/env python3
"""Record real X11 pixels while running a command. No generated simulation frames."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--display', default=':99.0')
    parser.add_argument('--size', default='1280x720')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('provide the mission command after --')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    raw = args.output_dir / 'raw.mkv'
    meta = {'schema_version': 1, 'display': args.display, 'size': args.size, 'capture_fps': 24,
            'capture_start_monotonic_s': time.monotonic(), 'capture_start_unix_s': time.time(),
            'command': command, 'source': 'ffmpeg x11grab: actual Gazebo GUI'}
    with (args.output_dir / 'capture.log').open('w') as log:
        recording = subprocess.Popen([
            'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'warning', '-n',
            '-f', 'x11grab', '-draw_mouse', '0', '-framerate', '24', '-video_size', args.size,
            '-i', args.display, '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '18',
            '-pix_fmt', 'yuv420p', str(raw)], stdout=log, stderr=log)
        mission = None
        try:
            time.sleep(3)
            if recording.poll() is not None:
                raise RuntimeError('screen recorder exited before mission; inspect capture.log')
            mission = subprocess.Popen(command, start_new_session=True)
            while mission.poll() is None:
                if recording.poll() is not None:
                    raise RuntimeError('screen recorder failed during mission')
                time.sleep(.2)
            meta['command_exit_code'] = mission.returncode
            time.sleep(1)
        finally:
            if mission is not None and mission.poll() is None:
                os.killpg(mission.pid, signal.SIGINT)
                try:
                    mission.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(mission.pid, signal.SIGKILL)
                    mission.wait()
            if recording.poll() is None:
                recording.send_signal(signal.SIGINT)
            try:
                recording.wait(timeout=20)
            except subprocess.TimeoutExpired:
                recording.kill()
                recording.wait()
            meta['capture_end_monotonic_s'] = time.monotonic()
            meta['recorder_exit_code'] = recording.returncode
            (args.output_dir / 'capture.json').write_text(json.dumps(meta, indent=2) + '\n')
    if meta.get('command_exit_code') != 0 or meta['recorder_exit_code'] not in (0, 255):
        raise SystemExit('recording/mission failed; raw recording retained, not accepted as successful demo')


if __name__ == '__main__':
    main()
