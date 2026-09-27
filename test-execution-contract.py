"""Check limits between knots and preparation/rejection rules without ROS or motion."""
import copy
import importlib.util
import json
from pathlib import Path
import yaml
directory = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('contract', directory / 'execution-contract.py')
contract = importlib.util.module_from_spec(spec); spec.loader.exec_module(contract)
urdf = directory / 'tests/fixtures/limits.urdf'
calibration = 'calib_synthetic_fixture'
raw = json.loads((directory / 'tests/fixtures/mock-trajectory.json').read_text())
prepared = contract.prepare(raw, urdf, calibration)
rejected = []
for name, change in [
    ('changed model', lambda t: t['physical_contract'].update(urdf_sha256='bad')),
    ('nonzero terminal velocity', lambda t: t['points'][-1]['velocities'].__setitem__(0,.01)),
    ('nonmonotonic time', lambda t: t['points'][1].update(time_from_start_s=0)),
    ('joint limit', lambda t: t['points'][1]['positions'].__setitem__(0,8.))]:
    bad = copy.deepcopy(prepared); change(bad)
    try: contract.validate(bad, urdf, calibration)
    except ValueError: rejected.append(name)
    else: raise AssertionError('Failed to reject '+name)
# Equal knot positions can still overshoot between knots due to derivatives.
points = [{'positions':[6.2]*6, 'velocities':[v]*6, 'accelerations':[0.]*6,
           'time_from_start_s':t} for t,v in [(0.,1.),(1.,-1.)]]
assert contract.spline_bounds(points)['max_position'][0] > 6.2831853
report = {'result':'PASS','rejected':rejected,'interior_spline_overshoot_detected':True,
    'prepared_duration_s':prepared['points'][-1]['time_from_start_s'],
    'bounds':contract.validate(prepared,urdf,calibration)}
(directory / 'physical-contract-dod.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
