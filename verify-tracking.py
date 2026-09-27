"""Compare actual controller desired feedback to the exact previewed polynomial."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

directory = Path(__file__).resolve().parent
twins = json.loads((directory / 'twins-dod.json').read_text())
plan_file = Path(twins['trajectory_file'])
trajectory = json.loads(plan_file.read_text())
digest = hashlib.sha256(plan_file.read_bytes()).hexdigest()
if not digest == twins['trajectory_sha256'] == json.loads((directory / 'mock-execute-dod.json').read_text())['trajectory_sha256'] == json.loads((directory / 'preview-attestation.json').read_text())['trajectory_sha256']:
    raise RuntimeError('Preview / execute / recorded plan hashes differ')
spec = importlib.util.spec_from_file_location('model', directory / 'demo-model.py')
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)
errors = []
with (directory / 'planned-vs-actual.csv').open() as stream:
    for row in csv.DictReader(stream):
        expected = model.sample_trajectory(trajectory, float(row['time_s']))
        reported = np.array([float(row['planned_'+name]) for name in trajectory['joint_names']])
        errors.append(float(np.max(np.abs(expected-reported))))
error = max(errors) if errors else float('inf')
report = {'result': 'PASS' if error < 1e-5 else 'FAIL', 'trajectory_sha256': digest,
    'samples': len(errors), 'controller_desired_vs_preview_interpolation_max_error_rad': error}
(directory / 'feedback-vs-plan-dod.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
if report['result'] != 'PASS':
    raise RuntimeError('Controller desired trajectory differs from preview')
