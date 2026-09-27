"""Exercise the production executor with actual ros2_control GenericSystem in domain 176."""
import hashlib
import importlib.util
import json
import os
import subprocess
import threading
import time
import uuid
from pathlib import Path
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene
from controller_manager_msgs.srv import ListControllers

directory = Path(__file__).resolve().parent
output = directory / 'robot-setup/executor-mock-tests'
output.mkdir(exist_ok=True)
if os.environ.get('ROS_DOMAIN_ID') != '176':
    raise RuntimeError('Test requires isolated mock domain 176')

def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, directory / filename)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module

contract = load('contract', 'execution-contract.py')
scene_tools = load('scene_tools', 'scene-fingerprint.py')
urdf = directory / 'robot-setup/ur5e-calibrated.urdf'
calibration_hash = yaml.safe_load((urdf.parent / 'ur5e-calibration.yaml').read_text())['kinematics']['hash']
rclpy.init()
node = rclpy.create_node('ur_executor_mock_test_scene')
state = {'positions': None}
def on_state(message):
    q = dict(zip(message.name, message.position))
    if all(n in q for n in contract.JOINTS):
        state['positions'] = [q[n] for n in contract.JOINTS]
node.create_subscription(JointState, '/joint_states', on_state, qos_profile_sensor_data)
scene = PlanningScene(name='executor-test-empty-scene')
def get_scene(request, response):
    response.scene = scene
    return response
node.create_service(GetPlanningScene, '/get_planning_scene', get_scene)
executor = SingleThreadedExecutor(); executor.add_node(node)
thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
summary = {'result': 'FAIL', 'hardware': 'GenericSystem', 'domain': 176, 'physical_trajectory_sent': False, 'tests': []}

def make_plan():
    q = state['positions'][:]
    end = q[:]; end[0] += .025; end[2] -= .02
    trajectory = contract.prepare({'joint_names': contract.JOINTS, 'points': [
        {'positions': p, 'velocities': [0.]*6, 'accelerations': [0.]*6, 'time_from_start_s': t}
        for p,t in [(q,0.), (end,3.)]]}, urdf, calibration_hash)
    trajectory['physical_contract']['scene_sha256'] = scene_tools.fingerprint(PlanningScene(name=scene.name))
    path = output / 'trajectory.json'; path.write_text(json.dumps(trajectory,indent=2)+'\n')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / 'physical-preview-attestation.json').write_text(json.dumps({'result':'PASS','trajectory_sha256':digest}))
    approval = {'execution_id': str(uuid.uuid4()), 'trajectory_sha256': digest,
        'operator_approved_physical': True, 'work_area_clear': True, 'approved_unix_s':time.time(),
        'source':'isolated GenericSystem test, no physical hardware'}
    (output / 'approval.json').write_text(json.dumps(approval))
    return path, approval

def launch(path, mode='--execute'):
    return subprocess.Popen(['/usr/bin/python3',str(directory/'execute-physical-plan.py'), mode,
        '--mock-test','--output-dir',str(output),'--trajectory',str(path),'--approval',str(output/'approval.json')],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

def complete(process):
    text,_ = process.communicate(timeout=40)
    (output/'latest-executor.log').write_text(text)
    report = json.loads((output/'physical-execute-dod.json').read_text())
    return process.returncode, report

def verify_inactive():
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=3):raise RuntimeError('No controller list')
    future = client.call_async(ListControllers.Request())
    deadline = time.monotonic()+5
    while not future.done():
        if time.monotonic()>deadline:raise TimeoutError('Controller list')
        time.sleep(.02)
    controllers = future.result().controller
    node.destroy_client(client)
    assert any(c.name=='joint_trajectory_controller' and c.state=='inactive' for c in controllers), 'Controller remains active'

try:
    deadline = time.monotonic()+20
    while state['positions'] is None:
        if time.monotonic()>deadline:raise TimeoutError('No isolated mock state')
        time.sleep(.05)
    path, approval = make_plan()
    code, report = complete(launch(path))
    (output/'success-dod.json').write_text(json.dumps(report,indent=2)+'\n')
    assert code==0 and report['result']=='PASS' and report['controller_inactive_after_execution'], report
    assert report['controller_preview_max_error_rad']<1e-7, report
    verify_inactive()
    summary['tests'].append('exact prepared spline execution, feedback comparison, controller cleanup')
    path, approval = make_plan()
    process = launch(path)
    deadline = time.monotonic()+15
    while True:
        if process.poll() is not None:raise RuntimeError('Executor ended before Stop test')
        status_path = output/'physical-execution-state.json'
        if status_path.exists():
            status = json.loads(status_path.read_text())
            if status.get('execution_id')==approval['execution_id'] and status.get('trajectory_time_s',0)>.4:break
        if time.monotonic()>deadline:raise TimeoutError('No running execution to cancel')
        time.sleep(.02)
    (output/'physical-stop-request.json').write_text(json.dumps({'execution_id':approval['execution_id']}))
    code, report = complete(process)
    (output/'stop-dod.json').write_text(json.dumps(report,indent=2)+'\n')
    assert code==0 and report['result']=='STOPPED' and report['controller_inactive_after_execution'],report
    verify_inactive()
    summary['tests'].append('Stop cancels own UUID and deactivates controller')
    path, approval = make_plan()
    (output/'physical-preview-attestation.json').write_text(json.dumps({'result':'PASS','trajectory_sha256':'wrong'}))
    code, report = complete(launch(path))
    assert code!=0 and report['result']=='FAIL' and report['physical_trajectory_sent'] is False,report
    verify_inactive()
    summary['tests'].append('changed preview rejected before controller activation')
    summary['result']='PASS'
finally:
    (directory/'physical-executor-mock-dod.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    executor.shutdown(); thread.join(timeout=3); node.destroy_node(); rclpy.shutdown()
