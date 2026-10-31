#!/usr/bin/env python3
"""Check the measured erasure evidence, retaining failures as separate trials."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from PIL import Image
from ink_model import initial_pixels

parser = argparse.ArgumentParser()
parser.add_argument('--run-dir', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
run = args.run_dir
summary = json.loads((run/'summary.json').read_text())
events = [json.loads(s) for s in (run/'events.jsonl').read_text().splitlines()]
samples = [json.loads(s) for s in (run/'tool_samples.jsonl').read_text().splitlines()]
planned = [e for e in events if e['event'] == 'cartesian_planned']
bands = [e for e in events if e['event'] == 'band_executed']
assert summary['success'] and len(bands) == 3 and len(planned) == 15
assert all(e['fraction'] >= .999 for e in planned)
assert len([e for e in events if e['event'] == 'policy_decision']) == 3
pixels = {p.id: p for p in initial_pixels()}
erased = set()
for s in samples:
    if not s['removed']:
        continue
    assert s['wipe_enabled']
    tip, q = s['contact_tip_xyz'], s['contact_q_xyzw']
    assert .512-1e-9 <= tip[0] <= .520001
    assert 1-2*(q[1]**2+q[2]**2) >= math.cos(math.radians(10))
    for name in s['removed']:
        assert name not in erased and name in pixels
        p = pixels[name]
        assert abs(p.y-tip[1]) <= .035 and abs(p.z-tip[2]) <= .0225
        erased.add(name)
before = sum(v > 0 for v in Image.open(run/'before_mask.png').getdata())
after = sum(v > 0 for v in Image.open(run/'after_mask.png').getdata())
assert len(erased) == 27 and before > 0 and after == 0
assert summary['erased_pixels'] == 27 and summary['residual_pixels'] == 0
all_trials = [json.loads(p.read_text()) for p in sorted(Path('artifacts').glob('wipe*/summary.json'))]
payload = {'selected_run': summary, 'cartesian_paths': len(planned),
           'all_path_fractions': [e['fraction'] for e in planned],
           'before_mask_pixels': before, 'after_mask_pixels': after,
           'mask_erasure_ratio': 1-after/before, 'unique_tf_triggered_erasures': len(erased),
           'limitations': 'geometric surrogate only; no force/pigment physics or real robot execution',
           'trials_during_development': all_trials,
           'sha256': {name: hashlib.sha256((run/name).read_bytes()).hexdigest()
                      for name in ('summary.json','events.jsonl','tool_samples.jsonl','before_mask.png','after_mask.png','transfer_paths.json')}}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({key: value for key, value in payload.items() if key not in ('trials_during_development','sha256')}, indent=2))
