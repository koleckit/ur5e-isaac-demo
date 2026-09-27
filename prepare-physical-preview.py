"""Read-only physical preflight and current-state seed for Isaac preview."""
import importlib.util
import json
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents

directory = Path(__file__).resolve().parent
if os.environ.get('ROS_DOMAIN_ID') != '174':
    raise RuntimeError('Physical preflight must use domain 174')
spec = importlib.util.spec_from_file_location('demo_model', directory / 'demo-model.py')
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)
rclpy.init()
node = rclpy.create_node('ur_physical_preview_preflight')
state = {'received': 0.0, 'positions': None, 'program': None}
def receive(message):
    mapped = dict(zip(message.name, message.position))
    if all(name in mapped and np.isfinite(mapped[name]) for name in model.JOINT_NAMES):
        state.update(positions=mapped, received=time.monotonic())
node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)
node.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
    lambda message: state.update(program=message.data),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
report = {'result': 'FAIL', 'physical_trajectory_sent': False}
try:
    futures = []
    for service, kind in (('/controller_manager/list_controllers', ListControllers),
                          ('/controller_manager/list_hardware_components', ListHardwareComponents)):
        client = node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=5):
            raise TimeoutError(service)
        futures.append(client.call_async(kind.Request()))
    deadline = time.monotonic()+10
    while not (all(f.done() for f in futures) and state['positions'] and state['program'] is not None):
        if time.monotonic() > deadline:
            raise TimeoutError('Missing physical state or controller response')
        rclpy.spin_once(node, timeout_sec=.05)
    if time.monotonic()-state['received'] > .2:
        raise RuntimeError('Physical joint state stale')
    hardware = futures[1].result().component
    if len(hardware) != 1 or hardware[0].plugin_name != 'ur_robot_driver/URPositionHardwareInterface':
        raise RuntimeError('Expected exactly one physical UR hardware component')
    controllers = futures[0].result().controller
    motion = {'joint_trajectory_controller', 'forward_velocity_controller', 'forward_position_controller',
              'forward_effort_controller', 'force_mode_controller', 'passthrough_trajectory_controller',
              'freedrive_mode_controller', 'motion_primitive_forward_controller'}
    if any(c.state == 'active' for c in controllers if c.name in motion):
        raise RuntimeError('Physical preview expects inactive motion controllers')
    if len(node.get_publishers_info_by_topic('/joint_states')) != 1:
        raise RuntimeError('Expected one physical joint state source')
    fk = json.loads((directory / 'robot-setup/calibrated-fk-dod.json').read_text())
    if fk['result'] != 'PASS' or max(abs(v) for v in fk['active_tcp_offset_rotvec']) > 1e-9:
        raise RuntimeError('Calibrated model must be validated with zero active TCP')
    names = model.JOINT_NAMES
    positions = [state['positions'][name] for name in names]
    matrix = model.Kinematics(directory / 'robot-setup/ur5e-calibrated.urdf').world_transforms(state['positions'])['tool0']
    target = {'frame_id': 'base_link', 'tcp_link': 'tool0', 'units': 'metres',
              'calibration': fk['calibration_hash'], 'matrix_column_vectors': matrix.tolist()}
    trajectory = {'joint_names': names, 'target_tcp': target, 'points': [
        {'positions': positions, 'velocities': [0.0]*6, 'accelerations': [0.0]*6,
         'time_from_start_s': seconds} for seconds in (0.0, 1.0)]}
    (directory / 'robot-setup/physical-seed-trajectory.json').write_text(json.dumps(trajectory, indent=2)+'\n')
    report.update(result='PASS', domain=174, hardware_plugin=hardware[0].plugin_name,
                  robot_program_running=state['program'], motion_controllers_inactive=True,
                  joint_positions_rad=positions, target_tcp=target)
finally:
    (directory / 'robot-setup/physical-preview-preflight-dod.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
