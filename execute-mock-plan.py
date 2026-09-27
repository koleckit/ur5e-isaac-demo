"""Prepare/execute only GenericSystem mock; record time-aligned FJT feedback."""
import argparse
import csv
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers

parser = argparse.ArgumentParser()
parser.add_argument('--prepare', action='store_true')
parser.add_argument('--trajectory', default='ur5e-moveit-cumotion-planned-trajectory.json')
args = parser.parse_args()
directory = Path(__file__).resolve().parent
path = Path(args.trajectory)
if not path.is_absolute():
    path = directory / path
trajectory_bytes = path.read_bytes()
trajectory = json.loads(trajectory_bytes)
digest = hashlib.sha256(trajectory_bytes).hexdigest()
names = trajectory['joint_names']
state = {'positions': None, 'description': None, 'received': 0.0}
feedback_rows = []
report = {'result': 'FAIL', 'hardware': 'mock', 'mode': 'prepare' if args.prepare else 'execute', 'trajectory_sha256': digest}
rclpy.init()
node = rclpy.create_node('ur_demo_mock_execute')

def on_state(message):
    mapped = dict(zip(message.name, message.position))
    if all(n in mapped and math.isfinite(mapped[n]) for n in names):
        state.update(positions=[mapped[n] for n in names], received=time.monotonic())

def on_description(message):
    state['description'] = message.data

def wait_until(predicate, timeout, reason):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(reason)
        rclpy.spin_once(node, timeout_sec=0.01)

def on_feedback(message):
    feedback = message.feedback
    order = [feedback.joint_names.index(name) for name in names]
    seconds = feedback.desired.time_from_start.sec + feedback.desired.time_from_start.nanosec/1e9
    desired = [feedback.desired.positions[i] for i in order]
    actual = [feedback.actual.positions[i] for i in order]
    feedback_rows.append([seconds]+desired+actual+[a-b for a,b in zip(actual, desired)])

try:
    subscription = node.create_subscription(JointState, '/joint_states', on_state, qos_profile_sensor_data)
    description_subscription = node.create_subscription(String, '/robot_description', on_description,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait_until(lambda: state['positions'] is not None and state['description'] is not None, 40, 'Missing mock state/URDF')
    plugins = [p.text.strip() for p in ET.fromstring(state['description']).findall('.//ros2_control/hardware/plugin')]
    if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
        raise RuntimeError(f'Execution is mock only: refusing hardware {plugins}')
    if node.count_publishers('/robot_description') != 1 or node.count_publishers('/joint_states') != 1:
        raise RuntimeError('Expected exactly one mock description and one joint-state source')
    controller_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    wait_until(controller_client.service_is_ready, 15, 'No controller manager')
    deadline = time.monotonic()+30
    while True:
        pending = controller_client.call_async(ListControllers.Request())
        wait_until(pending.done, 5, 'Controller state request timeout')
        if any(c.name == 'joint_trajectory_controller' and c.state == 'active' for c in pending.result().controller):
            break
        if time.monotonic() > deadline:
            raise TimeoutError('joint_trajectory_controller is not active')
        rclpy.spin_once(node, timeout_sec=0.05)
    client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
    wait_until(client.server_is_ready, 15, 'FollowJointTrajectory unavailable')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    if args.prepare:
        target = trajectory['points'][0]['positions']
        goal.trajectory.points = [JointTrajectoryPoint(positions=state['positions'], velocities=[0.0]*6,
            time_from_start=Duration(nanosec=100000000)),
            JointTrajectoryPoint(positions=target, velocities=[0.0]*6, time_from_start=Duration(sec=3))]
    else:
        attestation = json.loads((directory / 'preview-attestation.json').read_text())
        if attestation.get('result') != 'PASS' or attestation.get('trajectory_sha256') != digest:
            raise RuntimeError('No successful preview of this exact trajectory')
        if time.monotonic()-state['received'] > 0.2:
            raise RuntimeError('Initial joint state is stale')
        difference = max(abs(a-b) for a,b in zip(state['positions'], trajectory['points'][0]['positions']))
        if difference > 0.003:
            raise RuntimeError(f'Initial state differs from preview by {difference} rad')
        last_time = -1.0
        for point in trajectory['points']:
            seconds = point['time_from_start_s']
            if seconds <= last_time or not all(math.isfinite(q) for q in point['positions']):
                raise ValueError('Invalid trajectory positions or timestamps')
            last_time = seconds
            sec = int(seconds)
            nanosec = round((seconds-sec)*1e9)
            if nanosec == 1000000000:
                sec, nanosec = sec+1, 0
            goal.trajectory.points.append(JointTrajectoryPoint(positions=point['positions'],
                velocities=point['velocities'], accelerations=point['accelerations'],
                time_from_start=Duration(sec=sec, nanosec=nanosec)))
        target = trajectory['points'][-1]['positions']
        # Preserve every point, derivative and timestamp from the previewed file.
        goal.trajectory.header.stamp = node.get_clock().now().to_msg()
        goal.trajectory.header.stamp.sec += 1
    submitted = client.send_goal_async(goal, feedback_callback=on_feedback)
    wait_until(submitted.done, 10, 'No goal acceptance')
    handle = submitted.result()
    if not handle.accepted:
        raise RuntimeError('Mock rejected the trajectory')
    if not args.prepare:
        (directory / 'execution-state.json').write_text(json.dumps({'state': 'RUNNING',
            'trajectory_sha256': digest, 'start_unix_s': goal.trajectory.header.stamp.sec+goal.trajectory.header.stamp.nanosec/1e9})+'\n')
    result_future = handle.get_result_async()
    wait_until(result_future.done, 20, 'Trajectory execution timed out')
    result = result_future.result()
    if result.status != 4 or result.result.error_code != 0:
        raise RuntimeError(f'FJT failed: {result.status}, {result.result}')
    wait_until(lambda: max(abs(a-b) for a,b in zip(state['positions'], target)) < 0.003, 3, 'Endpoint error')
    if len(feedback_rows) < 10:
        raise RuntimeError('Not enough time-aligned feedback samples')
    report.update(result='PASS', action_status=result.status, action_error_code=result.result.error_code,
        feedback_samples=len(feedback_rows), hardware_plugins=plugins,
        final_max_error_rad=max(abs(a-b) for a,b in zip(state['positions'], target)))
    if not args.prepare:
        rows = [row for row in feedback_rows if 0 <= row[0] <= trajectory['points'][-1]['time_from_start_s']]
        errors = np.asarray([row[-6:] for row in rows])
        report['rmse_rad_by_joint'] = dict(zip(names, np.sqrt(np.mean(errors**2, axis=0)).tolist()))
        report['max_abs_error_rad_by_joint'] = dict(zip(names, np.max(np.abs(errors),axis=0).tolist()))
        report['comparison'] = 'controller desired vs actual at the same feedback timestamp; mock only'
        if float(np.max(np.abs(errors))) > 0.03:
            raise RuntimeError('Tracking error exceeds mock DoD threshold 0.03 rad')
        with (directory / 'planned-vs-actual.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s']+[f'{prefix}_{n}' for prefix in ['planned','actual','error'] for n in names])
            writer.writerows(rows)
    print(json.dumps(report, indent=2))
except Exception as exc:
    report.update(result='FAIL', error=str(exc))
    raise
finally:
    suffix = 'prepare' if args.prepare else 'execute'
    (directory / f'mock-{suffix}-dod.json').write_text(json.dumps(report, indent=2)+'\n')
    if not args.prepare:
        (directory / 'execution-state.json').write_text(json.dumps({'state': 'DONE',
            'result': report['result'], 'trajectory_sha256': digest})+'\n')
    node.destroy_node()
    rclpy.shutdown()
