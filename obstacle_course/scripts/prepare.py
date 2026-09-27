"""Generate run inputs and a complete bare-UR5e sphere model from NVIDIA's XRDF."""
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import struct
import numpy as np
from ament_index_python.packages import get_package_share_directory
import yaml

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'obstacle_course/src'))
from world import load

parser = argparse.ArgumentParser()
parser.add_argument('--scenario', default='single_wall')
parser.add_argument('--output-dir', required=True)
parser.add_argument('--vendor-xrdf')
parser.add_argument('--urdf', default=str(root / 'ur5e.urdf'))
args = parser.parse_args()
out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
course = load(root / 'obstacle_course/config/obstacle_course.yaml', args.scenario)
(out / 'target.json').write_text(json.dumps(course['goal'], indent=2)+'\n')
names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
(out / 'seed.json').write_text(json.dumps({'joint_names':names,'points':[
    {'positions':course['initial_joint_positions'],'velocities':[0.]*6,'accelerations':[0.]*6,'time_from_start_s':0.},
    {'positions':course['initial_joint_positions'],'velocities':[0.]*6,'accelerations':[0.]*6,'time_from_start_s':2.}],
    'target_tcp':course['goal']},indent=2)+'\n')
if args.vendor_xrdf:
    xrdf = yaml.safe_load(Path(args.vendor_xrdf).read_text())
    spheres = xrdf['geometry'][xrdf['collision']['geometry']]['spheres']
    # Bound every vertex of the actual fixed-base collision mesh.
    mesh = Path(get_package_share_directory('ur_description')) / 'meshes/ur5e/collision/base.stl'
    raw = mesh.read_bytes(); count = struct.unpack_from('<I', raw, 80)[0]
    vertices = np.asarray([struct.unpack_from('<9f', raw, 84+i*50+12) for i in range(count)]).reshape(-1,3)
    center = (vertices.min(axis=0)+vertices.max(axis=0))/2
    radius = float(np.ceil(np.linalg.norm(vertices-center,axis=1).max()*1000)/1000)
    spheres['base_link_inertia'] = [{'center':center.tolist(),'radius':radius}]
    xrdf['collision'].setdefault('buffer_distance', {})['base_link_inertia'] = .01
    # Only mechanically adjacent base/shoulder overlap is ignored.
    xrdf['self_collision'].setdefault('ignore', {})['base_link_inertia'] = ['shoulder_link']
    collision_links = [link.get('name') for link in ET.parse(args.urdf).getroot().findall('link') if link.find('collision') is not None]
    if set(collision_links)-set(spheres):
        raise RuntimeError('XRDF omits collision links: '+str(set(collision_links)-set(spheres)))
    (out / 'ur5e-course.xrdf').write_text(yaml.safe_dump(xrdf, sort_keys=False))
    (out / 'robot-geometry.json').write_text(json.dumps({'collision_links':collision_links,
        'sphere_links':list(spheres),'sphere_count':sum(len(v) for v in spheres.values()),
        'self_collision_geometry':xrdf['self_collision']['geometry'],
        'adjacent_pair_exclusions':xrdf['self_collision']['ignore']},indent=2)+'\n')
