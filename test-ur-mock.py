"""Validate /joint_states and FollowJointTrajectory only on isolated mock UR5e."""
import json
import math
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '172'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_STATIC_PEERS', None)
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers

project_dir = Path(__file__).resolve().parent
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
state = {'description': None, 'positions': None, 'samples': 0}
report = {'result': 'FAIL', 'hardware': 'mock', 'domain': 172}
launch_process = None
node = None

def joint_callback(msg):
    values = dict(zip(msg.name, msg.position))
    if all(name in values and math.isfinite(values[name]) for name in joint_names):
        state['positions'] = [values[name] for name in joint_names]
        state['samples'] += 1

def description_callback(msg):
    state['description'] = msg.data

def wait_until(predicate, timeout, reason):
    deadline = time.monotonic() + timeout
    while not predicate():
        if launch_process.poll() is not None:
            raise RuntimeError('Mock launch exited; inspect ur-mock-launch.log')
        if time.monotonic() >= deadline:
            raise TimeoutError(reason)
        rclpy.spin_once(node, timeout_sec=0.1)

try:
    with (project_dir / 'ur-mock-launch.log').open('w') as stream:
        launch_process = subprocess.Popen(['bash', str(project_dir / 'launch-ur-mock.sh')],
                                          stdout=stream, stderr=subprocess.STDOUT,
                                          start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('ur5e_mock_dod')
        subscriptions = [
            node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data),
            node.create_subscription(String, '/robot_description', description_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))]
        wait_until(lambda: state['description'] is not None and state['samples'] >= 10,
                   75, 'Missing description or ten valid six-joint samples')
        description = ET.fromstring(state['description'])
        plugins = [e.text.strip() for e in description.findall('.//ros2_control/hardware/plugin')]
        if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
            raise RuntimeError(f'Refusing trajectory test: hardware is not exclusively mock: {plugins}')
        # The action server exists during configuration, before activation.
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        wait_until(controllers.service_is_ready, 30, 'Controller manager service unavailable')
        activation_deadline = time.monotonic() + 30
        while True:
            listed = controllers.call_async(ListControllers.Request())
            wait_until(listed.done, 5, 'Controller list request timed out')
            controller_states = {c.name: c.state for c in listed.result().controller}
            if controller_states.get('joint_trajectory_controller') == 'active':
                break
            if time.monotonic() >= activation_deadline:
                raise TimeoutError(f'Trajectory controller not active: {controller_states}')
            rclpy.spin_once(node, timeout_sec=0.2)
        report['controller_states'] = controller_states
        action = ActionClient(node, FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')
        wait_until(action.server_is_ready, 30, 'FollowJointTrajectory action unavailable')
        start = list(state['positions'])
        target = list(start)
        target[0] += 0.05
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=start, velocities=[0.0]*6,
                                 time_from_start=Duration(sec=0, nanosec=100000000)),
            JointTrajectoryPoint(positions=target, velocities=[0.0]*6,
                                 time_from_start=Duration(sec=2))]
        accepted = action.send_goal_async(goal)
        wait_until(accepted.done, 10, 'No goal acceptance response')
        handle = accepted.result()
        if not handle.accepted:
            raise RuntimeError('Mock controller rejected trajectory')
        result_future = handle.get_result_async()
        wait_until(result_future.done, 15, 'Trajectory did not finish')
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError(f'Trajectory failed: status={result.status}, result={result.result}')
        wait_until(lambda: max(abs(a-b) for a,b in zip(state['positions'], target)) < 0.003,
                   5, 'Joint states do not match the trajectory target')
        report.update(result='PASS', hardware_plugins=plugins,
                      valid_joint_samples=state['samples'], joint_names=joint_names,
                      start_positions=start, target_positions=target,
                      actual_positions=state['positions'],
                      final_max_error_rad=max(abs(a-b) for a,b in zip(state['positions'], target)),
                      action_status=result.status, action_error_code=result.result.error_code)
        print(json.dumps(report, indent=2))
except Exception as exc:
    report['error'] = str(exc)
    raise
finally:
    (project_dir / 'ur-mock-dod.json').write_text(json.dumps(report, indent=2) + '\n')
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if launch_process is not None:
        try:
            os.killpg(launch_process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(launch_process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            launch_process.wait()
