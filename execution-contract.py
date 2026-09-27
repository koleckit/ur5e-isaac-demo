"""Physical trajectory preparation and analytic spline limits, without ROS or actuators."""
import copy
import hashlib
import math
import xml.etree.ElementTree as ET
import numpy as np

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
MAX_VELOCITY = 0.15
MAX_ACCELERATION = 0.30
MIN_DURATION = 2.0
MAX_DURATION = 120.0

def coefficients(left, right):
    dt = right['time_from_start_s']-left['time_from_start_s']
    p0, p1 = np.asarray(left['positions']), np.asarray(right['positions'])
    v0, v1 = np.asarray(left['velocities']), np.asarray(right['velocities'])
    a0, a1 = np.asarray(left['accelerations']), np.asarray(right['accelerations'])
    high = np.linalg.solve([[dt**3, dt**4, dt**5], [3*dt**2, 4*dt**3, 5*dt**4],
                           [6*dt, 12*dt**2, 20*dt**3]],
                          np.array([p1-p0-v0*dt-a0*dt**2/2, v1-v0-a0*dt, a1-a0]))
    return dt, np.vstack([p0, v0, a0/2, high])

def extrema(polynomial, dt):
    derivative = np.polynomial.polynomial.polyder(polynomial)
    candidates = [0.0, dt]
    for root in np.polynomial.polynomial.polyroots(derivative):
        if abs(root.imag) < 1e-8 and 0 <= root.real <= dt:
            candidates.append(float(root.real))
    values = np.polynomial.polynomial.polyval(candidates, polynomial)
    return float(np.min(values)), float(np.max(values))

def spline_bounds(points):
    lower, upper = np.full(6, np.inf), np.full(6, -np.inf)
    velocity, acceleration = 0.0, 0.0
    for left, right in zip(points, points[1:]):
        dt, matrix = coefficients(left, right)
        for joint in range(6):
            polynomial = matrix[:, joint]
            lo, hi = extrema(polynomial, dt)
            lower[joint], upper[joint] = min(lower[joint], lo), max(upper[joint], hi)
            v = np.polynomial.polynomial.polyder(polynomial)
            a = np.polynomial.polynomial.polyder(v)
            velocity = max(velocity, max(abs(x) for x in extrema(v, dt)))
            acceleration = max(acceleration, max(abs(x) for x in extrema(a, dt)))
    return {'min_position': lower.tolist(), 'max_position': upper.tolist(),
            'max_velocity_rad_s': velocity, 'max_acceleration_rad_s2': acceleration}

def validate_shape(trajectory):
    if trajectory.get('joint_names') != JOINTS:
        raise ValueError('Unexpected physical joint order')
    points = trajectory.get('points', [])
    if not 2 <= len(points) <= 20000:
        raise ValueError('Invalid trajectory point count')
    previous = -1.0
    for point in points:
        t = point['time_from_start_s']
        if not math.isfinite(t) or t <= previous or t < 0:
            raise ValueError('Trajectory timestamps must be finite and strictly increasing')
        previous = t
        for key in ('positions', 'velocities', 'accelerations'):
            if len(point[key]) != 6 or not all(math.isfinite(v) for v in point[key]):
                raise ValueError('Invalid '+key)
    if abs(points[0]['time_from_start_s']) > 1e-9:
        raise ValueError('Physical trajectory must start at time zero')

def prepare(trajectory, urdf_file, calibration_hash):
    result = copy.deepcopy(trajectory)
    validate_shape(result)
    for point in (result['points'][0], result['points'][-1]):
        point['velocities'] = [0.0]*6
        point['accelerations'] = [0.0]*6
    bounds = spline_bounds(result['points'])
    factor = max(1.0, bounds['max_velocity_rad_s']/MAX_VELOCITY,
                 math.sqrt(bounds['max_acceleration_rad_s2']/MAX_ACCELERATION),
                 MIN_DURATION/result['points'][-1]['time_from_start_s'])
    for point in result['points']:
        point['time_from_start_s'] *= factor
        point['velocities'] = [v/factor for v in point['velocities']]
        point['accelerations'] = [a/factor**2 for a in point['accelerations']]
    result['physical_contract'] = {'version': 1, 'calibration_hash': calibration_hash,
        'urdf_sha256': hashlib.sha256(urdf_file.read_bytes()).hexdigest(),
        'max_velocity_rad_s': MAX_VELOCITY, 'max_acceleration_rad_s2': MAX_ACCELERATION,
        'retiming_factor': factor, 'terminal_velocity_zero': True, 'tcp': 'tool0', 'tcp_offset': [0.0]*6}
    result['physical_contract']['bounds'] = validate(result, urdf_file, calibration_hash)
    return result

def validate(trajectory, urdf_file, calibration_hash):
    validate_shape(trajectory)
    contract = trajectory.get('physical_contract', {})
    if contract.get('version') != 1 or contract.get('calibration_hash') != calibration_hash:
        raise ValueError('Trajectory lacks matching physical calibration contract')
    if contract.get('urdf_sha256') != hashlib.sha256(urdf_file.read_bytes()).hexdigest():
        raise ValueError('Robot model changed after planning')
    points = trajectory['points']
    if not MIN_DURATION-1e-9 <= points[-1]['time_from_start_s'] <= MAX_DURATION:
        raise ValueError('Physical trajectory duration must be 2..120 seconds')
    if any(abs(v) > 1e-12 for point in (points[0], points[-1])
           for key in ('velocities', 'accelerations') for v in point[key]):
        raise ValueError('Physical trajectory must start and finish at rest')
    bounds = spline_bounds(points)
    if bounds['max_velocity_rad_s'] > MAX_VELOCITY+1e-6 or bounds['max_acceleration_rad_s2'] > MAX_ACCELERATION+1e-6:
        raise ValueError('Interpolated spline exceeds physical velocity/acceleration limits')
    limits = {j.get('name'): j.find('limit') for j in ET.parse(urdf_file).getroot().findall('joint')}
    for index, name in enumerate(JOINTS):
        limit = limits[name]
        if bounds['min_position'][index] < float(limit.get('lower'))-1e-7 or bounds['max_position'][index] > float(limit.get('upper'))+1e-7:
            raise ValueError('Spline exceeds joint limits: '+name)
    return bounds

def collision_sample_times(trajectory):
    points = trajectory['points']; times = [points[0]['time_from_start_s']]
    for left, right in zip(points, points[1:]):
        dt, matrix = coefficients(left, right)
        max_velocity = max(max(abs(v) for v in extrema(np.polynomial.polynomial.polyder(matrix[:, j]), dt)) for j in range(6))
        count = max(2, math.ceil(max_velocity*dt/0.02))
        times.extend(np.linspace(left['time_from_start_s'], right['time_from_start_s'], count+1)[1:].tolist())
    if len(times) > 5000:
        raise ValueError('Collision sampling workload exceeds demo limit')
    return times
