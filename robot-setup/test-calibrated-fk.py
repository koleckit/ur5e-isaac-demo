"""Validate calibrated URDF against same-packet joint/TCP state on read-only Primary 30011."""
import importlib.util
import json
import socket
import struct
import time
from pathlib import Path
import numpy as np
import yaml

directory = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('demo_model', directory.parent / 'demo-model.py')
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)
kin = model.Kinematics(directory / 'ur5e-calibrated.urdf')
generic = model.Kinematics(directory.parent / 'ur5e.urdf')

def pose_matrix(values):
    result = np.eye(4)
    result[:3, 3] = values[:3]
    vector = np.asarray(values[3:6])
    angle = np.linalg.norm(vector)
    if angle > 1e-12:
        result[:3, :3] = model.rotation(vector / angle, angle)
    return result

def errors(predicted, actual):
    return float(np.linalg.norm(predicted[:3,3]-actual[:3,3])), float(np.arccos(
        np.clip((np.trace(predicted[:3,:3].T@actual[:3,:3])-1)/2, -1, 1)))

def read_exact(connection, length):
    data = bytearray()
    while len(data) < length:
        part = connection.recv(length-len(data))
        if not part:
            raise ConnectionError('Robot closed read-only Primary connection')
        data.extend(part)
    return bytes(data)

rows = []
report = {'result': 'FAIL', 'robot_ip': '192.168.60.201', 'port': 30011,
          'source': 'same RobotState packet: q_actual, actual TCP, active TCP offset',
          'calibration_hash': yaml.safe_load((directory / 'ur5e-calibration.yaml').read_text())['kinematics']['hash'],
          'physical_trajectory_sent': False}
try:
    with socket.create_connection((report['robot_ip'], report['port']), timeout=3) as connection:
        connection.settimeout(3)
        deadline = time.monotonic()+15
        while len(rows) < 30 and time.monotonic() < deadline:
            header = read_exact(connection, 5)
            length, kind = struct.unpack('!IB', header)
            if not 5 <= length <= 1048576:
                raise ValueError('Invalid Primary packet size')
            payload = read_exact(connection, length-5)
            if kind != 16:
                continue
            parts = {}; offset = 0
            while offset < len(payload):
                size, subtype = struct.unpack_from('!IB', payload, offset)
                if size < 5 or offset+size > len(payload):
                    raise ValueError('Invalid Primary subpackage')
                parts[subtype] = payload[offset+5:offset+size]
                offset += size
            if 1 not in parts or 4 not in parts:
                continue
            if len(parts[1]) < 246 or len(parts[4]) < 96:
                raise ValueError('Incomplete joint or Cartesian data')
            positions = [struct.unpack_from('!d', parts[1], index*41)[0] for index in range(6)]
            cartesian = struct.unpack_from('!12d', parts[4])
            actual, tcp_offset = pose_matrix(cartesian[:6]), pose_matrix(cartesian[6:])
            state = dict(zip(model.JOINT_NAMES, positions))
            transforms = kin.world_transforms(state)
            predicted = np.linalg.inv(transforms['base']) @ transforms['tool0'] @ tcp_offset
            old = generic.world_transforms(state)
            generic_prediction = np.linalg.inv(old['base']) @ old['tool0'] @ tcp_offset
            p, r = errors(predicted, actual)
            gp, gr = errors(generic_prediction, actual)
            rows.append({'positions_rad': positions, 'actual_tcp_base_rotvec': list(cartesian[:6]),
                         'active_tcp_offset_rotvec': list(cartesian[6:]),
                         'position_error_m': p, 'orientation_error_rad': r,
                         'generic_position_error_m': gp, 'generic_orientation_error_rad': gr})
    if len(rows) < 20:
        raise RuntimeError('Insufficient robot samples')
    report.update(samples=len(rows), max_position_error_m=max(x['position_error_m'] for x in rows),
                  max_orientation_error_rad=max(x['orientation_error_rad'] for x in rows),
                  generic_max_position_error_m=max(x['generic_position_error_m'] for x in rows),
                  active_tcp_offset_rotvec=rows[-1]['active_tcp_offset_rotvec'])
    if report['max_position_error_m'] > 0.0001 or report['max_orientation_error_rad'] > 0.0001:
        raise RuntimeError('Calibrated FK differs from controller TCP by >0.1 mm / 0.0001 rad')
    report['result'] = 'PASS'
finally:
    report['measurements'] = rows
    (directory / 'calibrated-fk-dod.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'measurements'}, indent=2))
