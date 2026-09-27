"""One static-world schema shared by ROS collision objects and Isaac Sim visuals."""
import hashlib
import json
import math
import re
from pathlib import Path
import numpy as np
import yaml

def vector(value, count, label):
    if not isinstance(value, (list, tuple)) or len(value) != count or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
        raise ValueError('Invalid '+label)
    return [float(v) for v in value]

def quaternion(value):
    result = vector(value, 4, 'quaternion_xyzw')
    if abs(sum(v*v for v in result)-1) > 1e-5:
        raise ValueError('Quaternion must be normalized')
    return result

def matrix(position, orientation):
    x,y,z,w = orientation
    result = np.eye(4)
    result[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    result[:3,3] = position
    return result

def load(filename, scenario):
    config = yaml.safe_load(Path(filename).read_text())
    if config.get('version') != 1 or config.get('frame_id') != 'base_link':
        raise ValueError('Course version 1 in base_link is required')
    if scenario not in config['scenarios']:
        raise ValueError('Unknown obstacle scenario: '+scenario)
    selected = config['scenarios'][scenario]
    obstacles = []
    for name, box in selected['obstacles'].items():
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) or box.get('type') != 'box':
            raise ValueError('Only named box primitives are supported')
        size = vector(box['size'], 3, 'box size')
        if any(v <= 0 or v > 5 for v in size):
            raise ValueError('Box dimensions must be 0..5 metres')
        obstacles.append({'id':'oc_'+name, 'type':'box', 'position':vector(box['position'],3,'position'),
            'size':size, 'quaternion_xyzw':quaternion(box.get('quaternion_xyzw',[0,0,0,1])),
            'color':vector(box.get('color',[.2,.45,.75]),3,'color')})
    if not 1 <= len(obstacles) <= 32:
        raise ValueError('Course requires 1..32 boxes')
    geometry = {'version':1,'frame_id':config['frame_id'],'obstacles':sorted(obstacles,key=lambda o:o['id'])}
    digest = hashlib.sha256(json.dumps(geometry,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    goal = selected.get('goal', config['goal'])
    position = vector(goal['position'],3,'goal position')
    orientation = quaternion(goal['quaternion_xyzw'])
    return dict(geometry, scenario=scenario, geometry_sha256=digest,
        config_path=str(Path(filename).resolve()), label=selected.get('label',scenario),
        initial_joint_positions=vector(config['initial_joint_positions'],6,'mock start joints'),
        goal={'position_xyz':position,'quaternion_xyzw':orientation,'frame_id':'base_link',
              'tcp_link':'tool0','units':'metres','matrix_column_vectors':matrix(position,orientation).tolist()})

def signed_box_distance(point, box):
    """Euclidean signed point-to-oriented-box distance (not a robot collision test)."""
    transform = matrix(box['position'],box['quaternion_xyzw'])
    local = transform[:3,:3].T @ (np.asarray(point)-transform[:3,3])
    delta = np.abs(local)-np.asarray(box['size'])/2
    return float(np.linalg.norm(np.maximum(delta,0))+min(max(delta),0))
