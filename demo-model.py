"""Shared URDF kinematics and timestamped trajectory interpolation (metres/radians)."""
import bisect
import math
import xml.etree.ElementTree as ET
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)

class Kinematics:
    def __init__(self, urdf):
        robot = ET.parse(urdf).getroot()
        self.joints = []
        for joint in robot.findall('joint'):
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = rotation([0,0,1], rpy[2]) @ rotation([0,1,0], rpy[1]) @ rotation([1,0,0], rpy[0])
            axis = joint.find('axis')
            self.joints.append(dict(name=joint.get('name'), kind=joint.get('type'),
                parent=joint.find('parent').get('link'), child=joint.find('child').get('link'),
                origin=transform, axis=[float(v) for v in axis.get('xyz', '1 0 0').split()] if axis is not None else [1,0,0]))
        children = {j['child'] for j in self.joints}
        self.root = next(l.get('name') for l in robot.findall('link') if l.get('name') not in children)

    def local_transforms(self, positions):
        result = {}
        for joint in self.joints:
            local = joint['origin'].copy()
            if joint['kind'] in ('revolute', 'continuous'):
                turn = np.eye(4)
                turn[:3, :3] = rotation(joint['axis'], positions[joint['name']])
                local = local @ turn
            result[joint['child']] = local
        return result

    def world_transforms(self, positions):
        local = self.local_transforms(positions)
        result = {self.root: np.eye(4)}
        pending = list(self.joints)
        while pending:
            progressed = False
            for joint in pending[:]:
                if joint['parent'] in result:
                    result[joint['child']] = result[joint['parent']] @ local[joint['child']]
                    pending.remove(joint)
                    progressed = True
            if not progressed:
                raise ValueError('URDF is not a connected kinematic tree')
        return result

def sample_trajectory(trajectory, seconds):
    points = trajectory['points']
    times = [p['time_from_start_s'] for p in points]
    if seconds <= times[0]:
        return np.asarray(points[0]['positions'])
    if seconds >= times[-1]:
        return np.asarray(points[-1]['positions'])
    index = bisect.bisect_right(times, seconds)-1
    left, right = points[index:index+2]
    duration = times[index+1]-times[index]
    elapsed = seconds-times[index]
    p0, p1 = np.asarray(left['positions']), np.asarray(right['positions'])
    if not left.get('velocities') or not right.get('velocities'):
        return p0 + (p1-p0)*elapsed/duration
    v0, v1 = np.asarray(left['velocities']), np.asarray(right['velocities'])
    if not left.get('accelerations') or not right.get('accelerations'):
        coefficients = np.linalg.solve([[duration**2, duration**3], [2*duration, 3*duration**2]],
            np.array([p1-p0-v0*duration, v1-v0]))
        return p0+v0*elapsed+coefficients[0]*elapsed**2+coefficients[1]*elapsed**3
    a0, a1 = np.asarray(left['accelerations']), np.asarray(right['accelerations'])
    coefficients = np.linalg.solve([[duration**3, duration**4, duration**5],
        [3*duration**2, 4*duration**3, 5*duration**4], [6*duration, 12*duration**2, 20*duration**3]],
        np.array([p1-p0-v0*duration-a0*duration**2/2, v1-v0-a0*duration, a1-a0]))
    return p0+v0*elapsed+a0*elapsed**2/2+sum(coefficients[k]*elapsed**(k+3) for k in range(3))
