"""Execute an operator-approved, immutable preview; own and clean up only this goal/controller."""
import argparse
import csv
import fcntl
import hashlib
import importlib.util
import json
import math
import os
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SwitchController
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from unique_identifier_msgs.msg import UUID
from action_msgs.srv import CancelGoal
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import GetParameters

parser = argparse.ArgumentParser()
mode = parser.add_mutually_exclusive_group(required=True)
mode.add_argument('--check-only', action='store_true')
mode.add_argument('--execute', action='store_true')
parser.add_argument('--trajectory', required=True)
parser.add_argument('--approval')
parser.add_argument('--mock-test', action='store_true')
parser.add_argument('--output-dir')
args = parser.parse_args()
directory = Path(__file__).resolve().parent
output = Path(args.output_dir).resolve() if args.output_dir else directory
expected_domain = '176' if args.mock_test else '174'
if os.environ.get('ROS_DOMAIN_ID') != expected_domain:
    raise RuntimeError('Wrong execution ROS domain; expected '+expected_domain)
if args.mock_test and output == directory:
    raise RuntimeError('Mock test must use its own output directory')
output.mkdir(parents=True, exist_ok=True)
lock = (output / 'physical-execution.lock').open('w')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, directory / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

contract = load_module('execution_contract', 'execution-contract.py')
model = load_module('demo_model', 'demo-model.py')
scene_tools = load_module('scene_fingerprint', 'scene-fingerprint.py')
urdf = directory / 'robot-setup/ur5e-calibrated.urdf'
calibration_hash = yaml.safe_load((urdf.parent / 'ur5e-calibration.yaml').read_text())['kinematics']['hash']
trajectory_bytes = Path(args.trajectory).read_bytes()
trajectory = json.loads(trajectory_bytes)
digest = hashlib.sha256(trajectory_bytes).hexdigest()
names = contract.JOINTS
execution_id = str(uuid.uuid4())
if args.approval:
    approval = json.loads(Path(args.approval).read_text())
    execution_id = str(uuid.UUID(approval['execution_id']))
else:
    approval = {}
goal_uuid = UUID(uuid=list(uuid.UUID(execution_id).bytes))
rows, handle, action, result_future = [], None, None, None
activation_requested, submitted, stop_requested = False, False, False
state = {'received': 0.0, 'positions': None, 'velocities': None, 'description': None,
         'program': True if args.mock_test else None, 'robot_mode': 7 if args.mock_test else None,
         'safety_mode': 1 if args.mock_test else None, 'speed': None, 'speed_received': 0.0}
display = {'state': 'CHECKING', 'execution_id': execution_id, 'trajectory_sha256': digest,
           'trajectory_time_s': 0.0}
report = {'result': 'FAIL', 'mode': 'check-only' if args.check_only else 'execute',
          'hardware': 'mock' if args.mock_test else 'physical UR5e', 'execution_id': execution_id,
          'trajectory_sha256': digest, 'physical_trajectory_sent': False}
run_dir = None
rclpy.init()
node = rclpy.create_node('ur_demo_physical_execute')

def atomic_json(path, value):
    temporary = path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(value, indent=2)+'\n')
    temporary.replace(path)

def emit(**values):
    display.update(values)
    if args.execute:
        course = trajectory.get('obstacle_course')
        if course and (not course.get('enabled') or approval.get('physical_obstacles_match_scene') is not True
                or approval.get('obstacle_geometry_sha256') != course['geometry_sha256']):
            raise RuntimeError('Confirm that physical obstacles match the previewed course')
        atomic_json(output / 'physical-execution-state.json', display)

def signal_stop(signum, frame):
    global stop_requested
    stop_requested = True

signal.signal(signal.SIGINT, signal_stop)
signal.signal(signal.SIGTERM, signal_stop)

def on_state(message):
    positions = dict(zip(message.name, message.position))
    velocities = dict(zip(message.name, message.velocity))
    if all(n in positions and math.isfinite(positions[n]) for n in names):
        state.update(positions=[positions[n] for n in names],
                     velocities=[velocities.get(n, float('nan')) for n in names], received=time.monotonic())

def on_speed(message):
    # UR SpeedScalingStateBroadcaster publishes percent (hardware/JTC use 0..1).
    state.update(speed=message.data/100.0, speed_received=time.monotonic())

def wait_until(predicate, timeout, reason):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(reason)
        rclpy.spin_once(node, timeout_sec=.01)

def rpc(kind, service, request=None, timeout=10):
    client = node.create_client(kind, service)
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise TimeoutError('Service unavailable: '+service)
        future = client.call_async(request or kind.Request())
        wait_until(future.done, timeout, 'Service timeout: '+service)
        return future.result()
    finally:
        node.destroy_client(client)

def controllers():
    return rpc(ListControllers, '/controller_manager/list_controllers').controller

def check_live(require_start=False):
    if time.monotonic()-state['received'] > .25:
        raise RuntimeError('Joint state stale; Plan again after reconnecting')
    if state['program'] is not True or state['robot_mode'] != 7 or state['safety_mode'] not in (1, 2):
        raise RuntimeError('External Control / robot mode / safety mode does not permit execution')
    if time.monotonic()-state['speed_received'] > .5 or not math.isfinite(state['speed']) or not 0 <= state['speed'] <= 1.001:
        raise RuntimeError('Speed scaling unavailable or invalid')
    if require_start:
        difference = max(abs(a-b) for a,b in zip(state['positions'], trajectory['points'][0]['positions']))
        if difference > .003:
            raise RuntimeError(f'Robot moved since planning ({difference:.5f} rad); Plan and Preview again')
        if not all(math.isfinite(v) and abs(v) < .02 for v in state['velocities']):
            raise RuntimeError('Robot must be stationary before Execute')

def stop_file_requested():
    path = output / 'physical-stop-request.json'
    if not path.exists():
        return False
    try:
        return json.loads(path.read_text()).get('execution_id') == execution_id
    except (OSError, ValueError):
        return False

def on_feedback(message):
    feedback = message.feedback
    order = [feedback.joint_names.index(n) for n in names]
    seconds = feedback.desired.time_from_start.sec+feedback.desired.time_from_start.nanosec/1e9
    desired = [feedback.desired.positions[i] for i in order]
    actual = [feedback.actual.positions[i] for i in order]
    planned = model.sample_trajectory(trajectory, seconds).tolist()
    errors = [a-p for a,p in zip(actual, planned)]
    rows.append([seconds, time.time(), state['speed']]+planned+desired+actual+errors)
    emit(state='RUNNING', trajectory_time_s=seconds, speed_scaling=state['speed'],
         max_tracking_error_rad=max(abs(x) for x in errors), actual_positions=actual,
         desired_positions=desired)

def switch(active):
    request = SwitchController.Request()
    request.activate_controllers = ['joint_trajectory_controller'] if active else []
    request.deactivate_controllers = [] if active else ['joint_trajectory_controller']
    request.strictness = SwitchController.Request.STRICT
    request.activate_asap = True
    request.timeout = Duration(sec=5)
    response = rpc(SwitchController, '/controller_manager/switch_controller', request)
    if not response.ok:
        raise RuntimeError('Controller switch failed: '+response.message)

def duration_message(seconds):
    nanoseconds = round(seconds*1e9)
    return Duration(sec=nanoseconds//1000000000, nanosec=nanoseconds%1000000000)

try:
    contract.validate(trajectory, urdf, calibration_hash)
    attestation = json.loads((output / 'physical-preview-attestation.json').read_text())
    if attestation.get('result') != 'PASS' or attestation.get('trajectory_sha256') != digest:
        raise RuntimeError('This exact prepared trajectory has not completed Preview')
    if args.execute:
        if approval.get('trajectory_sha256') != digest or approval.get('operator_approved_physical') is not True:
            raise RuntimeError('Execute requires the operator approval of this exact preview')
        if approval.get('work_area_clear') is not True or not 0 <= time.time()-approval.get('approved_unix_s', 0) <= 30:
            raise RuntimeError('Approval expired or work area not confirmed')
        run_dir = output / 'execution-history' / execution_id
        run_dir.mkdir(parents=True, exist_ok=False)
        (run_dir / 'trajectory.json').write_bytes(trajectory_bytes)
        atomic_json(run_dir / 'approval.json', approval)
        atomic_json(run_dir / 'preview.json', attestation)
    node.create_subscription(JointState, '/joint_states', on_state, qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/robot_description', lambda m: state.update(description=m.data), latched)
    node.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling', on_speed, qos_profile_sensor_data)
    if not args.mock_test:
        node.create_subscription(Bool, '/io_and_status_controller/robot_program_running', lambda m: state.update(program=m.data), latched)
        node.create_subscription(RobotMode, '/io_and_status_controller/robot_mode', lambda m: state.update(robot_mode=m.mode), latched)
        node.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode', lambda m: state.update(safety_mode=m.mode), latched)
    wait_until(lambda: state['positions'] is not None and state['description'] is not None and state['speed'] is not None
               and state['program'] is not None and state['safety_mode'] is not None and state['robot_mode'] is not None,
               10, 'Missing robot state or modes')
    hardware = rpc(ListHardwareComponents, '/controller_manager/list_hardware_components').component
    expected_plugin = 'mock_components/GenericSystem' if args.mock_test else 'ur_robot_driver/URPositionHardwareInterface'
    plugins = [p.text.strip() for p in ET.fromstring(state['description']).findall('.//ros2_control/hardware/plugin')]
    if len(hardware) != 1 or hardware[0].plugin_name != expected_plugin or plugins != [expected_plugin]:
        raise RuntimeError('Refusing unexpected hardware: '+str(plugins))
    if hardware[0].state.label != 'active':
        raise RuntimeError('Hardware is not active')
    if node.count_publishers('/joint_states') != 1 or node.count_publishers('/robot_description') != 1:
        raise RuntimeError('Expected exactly one state source and robot description')
    if not args.mock_test:
        parameters = {p.get('name'): p.text for p in ET.fromstring(state['description']).findall('.//ros2_control/hardware/param')}
        if parameters.get('robot_ip') != '192.168.60.201' or parameters.get('kinematics/hash') != calibration_hash:
            raise RuntimeError('Driver IP/calibration differs from the planned robot')
        check = subprocess.run(['/usr/bin/python3', str(directory / 'robot-setup/test-calibrated-fk.py')],
                               capture_output=True, text=True, timeout=20)
        if check.returncode:
            raise RuntimeError('Read-only calibrated TCP check failed: '+check.stderr[-300:])
        fk = json.loads((directory / 'robot-setup/calibrated-fk-dod.json').read_text())
        if fk['result'] != 'PASS' or max(abs(v) for v in fk['active_tcp_offset_rotvec']) > 1e-9:
            raise RuntimeError('This demo requires zero active TCP and the calibrated bare robot')
    scene_request = GetPlanningScene.Request()
    scene_request.components.components = scene_tools.COMPONENTS
    scene = rpc(GetPlanningScene, '/get_planning_scene', scene_request).scene
    if scene_tools.fingerprint(scene) != trajectory['physical_contract'].get('scene_sha256'):
        raise RuntimeError('Collision scene changed after planning; Plan again')
    current = controllers()
    motion_controllers = {'joint_trajectory_controller', 'forward_velocity_controller', 'forward_position_controller',
        'forward_effort_controller', 'force_mode_controller', 'passthrough_trajectory_controller',
        'freedrive_mode_controller', 'motion_primitive_forward_controller'}
    if any(c.name in motion_controllers and c.state == 'active' for c in current):
        raise RuntimeError('A motion controller is already active; refusing to take over')
    if any(c.state == 'active' and c.claimed_interfaces for c in current):
        # Broadcasters and UR IO/status controllers claim non-joint interfaces.
        if any(c.state == 'active' and any(i.split('/')[0] in names for i in c.claimed_interfaces) for c in current):
            raise RuntimeError('Another joint motion controller is active')
    if not any(c.name == 'joint_trajectory_controller' and c.state == 'inactive' for c in current):
        raise RuntimeError('Expected an inactive joint_trajectory_controller')
    parameter_request = GetParameters.Request(names=['speed_scaling.state_interface', 'interpolation_method'])
    parameters = rpc(GetParameters, '/joint_trajectory_controller/get_parameters', parameter_request).values
    if parameters[0].string_value != 'speed_scaling/speed_scaling_factor' or parameters[1].string_value != 'splines':
        raise RuntimeError('Controller must use the UR speed scaling state and spline interpolation')
    report['controller_speed_scaling_interface'] = parameters[0].string_value
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=.01)
    check_live(require_start=True)
    report.update(preflight='PASS', hardware_plugins=plugins, robot_mode=state['robot_mode'],
                  safety_mode=state['safety_mode'], speed_scaling=state['speed'],
                  initial_max_error_rad=max(abs(a-b) for a,b in zip(state['positions'], trajectory['points'][0]['positions'])))
    if args.execute:
        if stop_requested or stop_file_requested():
            raise InterruptedError('Stop before activation')
        emit(state='ARMING')
        activation_requested = True
        switch(True)
        action = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        wait_until(action.server_is_ready, 5, 'FollowJointTrajectory unavailable')
        check_live(require_start=True)
        if stop_requested or stop_file_requested():
            raise InterruptedError('Stop before goal submission')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for p in trajectory['points']:
            goal.trajectory.points.append(JointTrajectoryPoint(positions=p['positions'], velocities=p['velocities'],
                accelerations=p['accelerations'], time_from_start=duration_message(p['time_from_start_s'])))
        goal.path_tolerance = [JointTolerance(name=n, position=.05, velocity=-1.0, acceleration=-1.0) for n in names]
        goal.goal_tolerance = [JointTolerance(name=n, position=.003, velocity=.02, acceleration=-1.0) for n in names]
        goal.goal_time_tolerance = Duration(sec=3)
        goal.trajectory.header.stamp = node.get_clock().now().to_msg()
        goal.trajectory.header.stamp.sec += 1
        submitted = True
        report['physical_trajectory_sent'] = not args.mock_test
        pending = action.send_goal_async(goal, goal_uuid=goal_uuid, feedback_callback=on_feedback)
        wait_until(pending.done, 5, 'Goal acceptance timeout')
        handle = pending.result()
        if not handle.accepted:
            raise RuntimeError('Controller rejected the trajectory')
        result_future = handle.get_result_async()
        emit(state='QUEUED')
        deadline, paused_since = time.monotonic()+600, None
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=.01)
            check_live()
            if stop_requested or stop_file_requested():
                raise InterruptedError('Operator Stop / application closing')
            if rows and display.get('max_tracking_error_rad', 0) > .05:
                raise RuntimeError('Tracking error exceeds 0.05 rad')
            if state['speed'] < .001:
                paused_since = paused_since or time.monotonic()
                if time.monotonic()-paused_since > 30:
                    raise RuntimeError('Speed scaling paused for more than 30 seconds')
            else:
                paused_since = None
            if time.monotonic() > deadline:
                raise TimeoutError('Execution exceeded 600 seconds')
        result = result_future.result()
        report.update(action_status=result.status, action_error_code=result.result.error_code,
                      action_message=result.result.error_string)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError('FollowJointTrajectory failed: '+str(result.result))
        wait_until(lambda: max(abs(a-b) for a,b in zip(state['positions'], trajectory['points'][-1]['positions'])) < .003,
                   3, 'Final robot position outside goal tolerance')
        if len(rows) < 2:
            raise RuntimeError('Insufficient controller feedback for planned-vs-actual')
        report['final_max_error_rad'] = max(abs(a-b) for a,b in zip(state['positions'], trajectory['points'][-1]['positions']))
    report['result'] = 'PASS'
except Exception as exc:
    report.update(result='STOPPED' if isinstance(exc, InterruptedError) else 'FAIL', error=str(exc))
finally:
    if submitted and (result_future is None or not result_future.done()):
        try:
            # Cancel only our UUID, including a goal whose acceptance response was lost.
            request = CancelGoal.Request()
            request.goal_info.goal_id = goal_uuid
            cancellation = rpc(CancelGoal, '/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal', request)
            report['cancel_return_code'] = cancellation.return_code
            if result_future is not None:
                wait_until(result_future.done, 5, 'Canceled goal did not finish')
        except Exception as exc:
            report['cancel_error'] = str(exc)
    if activation_requested:
        try:
            if any(c.name == 'joint_trajectory_controller' and c.state == 'active' for c in controllers()):
                switch(False)
            report['controller_inactive_after_execution'] = any(
                c.name == 'joint_trajectory_controller' and c.state == 'inactive' for c in controllers())
            if not report['controller_inactive_after_execution']:
                raise RuntimeError('Motion controller did not deactivate')
        except Exception as exc:
            report.update(result='FAIL', cleanup_error=str(exc))
    if rows:
        errors = np.asarray([row[-6:] for row in rows])
        report.update(feedback_samples=len(rows), comparison='preview spline vs actual at controller scaled trajectory time',
            rmse_rad_by_joint=dict(zip(names, np.sqrt(np.mean(errors**2, axis=0)).tolist())),
            max_abs_error_rad_by_joint=dict(zip(names, np.max(np.abs(errors), axis=0).tolist())),
            controller_preview_max_error_rad=float(np.max(np.abs(np.asarray([r[3:9] for r in rows])-np.asarray([r[9:15] for r in rows])))))
        with (output / 'physical-planned-vs-actual.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['trajectory_time_s', 'unix_s', 'speed_scaling']+
                [p+'_'+n for p in ['planned', 'controller_desired', 'actual', 'error'] for n in names])
            writer.writerows(rows)
        if run_dir:
            (run_dir / 'planned-vs-actual.csv').write_bytes((output / 'physical-planned-vs-actual.csv').read_bytes())
    suffix = 'preflight' if args.check_only else 'execute'
    atomic_json(output / f'physical-{suffix}-dod.json', report)
    if run_dir:
        atomic_json(run_dir / 'result.json', report)
    emit(state='DONE' if report['result'] == 'PASS' else report['result'], result=report['result'],
         error=report.get('cleanup_error', report.get('error', '')))
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    lock.close()
if report['result'] not in ('PASS', 'STOPPED'):
    raise SystemExit(1)
