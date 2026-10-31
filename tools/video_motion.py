#!/usr/bin/env python3
"""Convert measured tool TF samples into the shared motion-only editor format."""
import argparse
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--samples', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--skip-first-seconds', type=float, default=0., help='omit initial GUI warmup, never an erasure event')
args = parser.parse_args()
records = [json.loads(line) for line in args.samples.read_text().splitlines()]
if not records or not 0 <= args.skip_first_seconds <= 10:
    raise ValueError('need samples and a warmup cutoff between0 and10seconds')
cutoff = records[0]['wall_monotonic_s'] + args.skip_first_seconds
if any(r['removed'] and r['wall_monotonic_s'] < cutoff for r in records):
    raise ValueError('warmup cut would hide a recorded erasure')
output, previous = [], None
for record in records:
    moving = False
    if previous:
        dt = record['wall_monotonic_s'] - previous['wall_monotonic_s']
        distance = math.dist(record['ee_xyz'], previous['ee_xyz'])
        dot = abs(sum(a*b for a, b in zip(record['ee_q_xyzw'], previous['ee_q_xyzw'])))
        angle = 2*math.acos(min(1., dot))
        moving = 0 < dt <= .3 and (distance/dt > .003 or angle/dt > .02)
    output.append({'wall_monotonic_s': record['wall_monotonic_s'],
                   'moving': moving and record['wall_monotonic_s'] >= cutoff})
    previous = record
args.output.write_text(''.join(json.dumps(r) + '\n' for r in output))
