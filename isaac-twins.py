"""REAL/GHOST display; physical goals are sent only by the guarded host executor."""
import argparse
import hashlib
import importlib.util
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--test', action='store_true')
parser.add_argument('--workflow-test', action='store_true')
parser.add_argument('--physical-preview', action='store_true')
parser.add_argument('--enable-physical-execution', action='store_true')
parser.add_argument('--plan-preview-test', action='store_true')
parser.add_argument('--trajectory')
args = parser.parse_args()
if args.enable_physical_execution and not args.physical_preview:
    raise ValueError('Physical Execute requires physical mode')
if args.physical_preview and args.workflow_test:
    raise ValueError('Physical mode supports planning and preview only')
if args.physical_preview and os.environ.get('ROS_DOMAIN_ID') != '174':
    raise ValueError('Physical preview requires the dedicated ROS domain 174')
directory = Path(__file__).resolve().parent
prefix = 'physical-' if args.physical_preview else ''
args.trajectory = args.trajectory or ('robot-setup/physical-seed-trajectory.json' if args.physical_preview else 'ur5e-moveit-cumotion-planned-trajectory.json')
urdf_file = directory / ('robot-setup/ur5e-calibrated.urdf' if args.physical_preview else 'ur5e.urdf')
plan_prefix = 'physical-interactive-cumotion' if args.physical_preview else 'interactive-cumotion'
from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'renderer': 'RayTracedLighting'})
shutdown_requested = False
def on_shutdown(signum, frame):
    global shutdown_requested
    shutdown_requested = True
signal.signal(signal.SIGINT, on_shutdown)
signal.signal(signal.SIGTERM, on_shutdown)
import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

module_spec = importlib.util.spec_from_file_location('demo_model', directory / 'demo-model.py')
model = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(model)
trajectory_path = Path(args.trajectory)
if not trajectory_path.is_absolute():
    trajectory_path = directory / trajectory_path
trajectory_bytes = trajectory_path.read_bytes()
trajectory = json.loads(trajectory_bytes)
trajectory_hash = hashlib.sha256(trajectory_bytes).hexdigest()
names = trajectory['joint_names']
duration = trajectory['points'][-1]['time_from_start_s']
kinematics = model.Kinematics(urdf_file)
report = {'result': 'FAIL', 'source': 'ROS /joint_states', 'display': 'kinematic, no physics control',
          'trajectory_sha256': trajectory_hash, 'physical_calibration': 'factory calibrated' if args.physical_preview else 'pending robot',
          'physical_preview': args.physical_preview, 'physical_trajectory_sent': False}
node = None
operation = None
operation_log = None

def usd_matrix(matrix):
    # USD uses row vectors; URDF/FK uses column vectors.
    return Gf.Matrix4d(*matrix.T.flatten().tolist())

try:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate('isaacsim.ros2.nodes', True)
    app.update()
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, Float64
    rclpy.init()
    node = rclpy.create_node('ur_demo_isaac_twins')
    state = {'positions': None, 'samples': 0, 'received': 0.0, 'stamp': 0.0, 'ages': []}
    def on_state(message):
        incoming = dict(zip(message.name, message.position))
        if all(n in incoming and np.isfinite(incoming[n]) for n in names):
            state.update(positions={n: incoming[n] for n in names}, samples=state['samples']+1,
                         received=time.monotonic(), stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9)
            if state['stamp'] > 0:
                state['ages'].append(time.time()-state['stamp'])
    subscription = node.create_subscription(JointState, '/joint_states', on_state, qos_profile_sensor_data)
    robot_status = {'program': False, 'speed': 0.0}
    if args.physical_preview:
        node.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
            lambda m: robot_status.update(program=m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
            lambda m: robot_status.update(speed=m.data/100.0), qos_profile_sensor_data)
    mirror_publisher = node.create_publisher(JointState, '/ur_demo/real_mirror', 10)
    target_publisher = node.create_publisher(PoseStamped, '/ur_demo/target_tcp', 10)
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    asset = json.loads((directory / ('ur5e-physical-asset.json' if args.physical_preview else 'ur5e-asset.json')).read_text())['usd_path']
    twin_maps = {}
    for twin in ['REAL', 'GHOST']:
        prim = stage.DefinePrim('/World/'+twin, 'Xform')
        prim.GetReferences().AddReference(asset)
        prim.GetVariantSet('Physics').SetVariantSelection('none')
        link_map = {}
        for link in [kinematics.root] + [j['child'] for j in kinematics.joints]:
            parent_link = next((j['parent'] for j in kinematics.joints if j['child'] == link), None)
            matches = [p for p in Usd.PrimRange(prim) if p.GetName() == link and p.IsA(UsdGeom.Xform)
                       and (parent_link is None or p.GetParent().GetName() == parent_link)]
            if len(matches) != 1:
                raise ValueError(f'{twin}: expected one visual prim for link {link}: {matches}')
            xform = UsdGeom.Xformable(matches[0])
            xform.ClearXformOpOrder()
            link_map[link] = xform.AddTransformOp(opSuffix='live')
        twin_maps[twin] = link_map
    ghost_material = UsdShade.Material.Define(stage, '/World/GhostMaterial')
    shader = UsdShade.Shader.Define(stage, '/World/GhostMaterial/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.55, 0.05))
    shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(0.35)
    ghost_material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/GHOST')).Bind(
        ghost_material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(1500.0)
    target = UsdGeom.Xform.Define(stage, '/World/TargetTCP')
    target_op = target.AddTransformOp()
    target_matrix = (np.asarray(trajectory['target_tcp']['matrix_column_vectors']) if 'target_tcp' in trajectory
        else kinematics.world_transforms(dict(zip(names, trajectory['points'][-1]['positions'])))['tool0'])
    target_op.Set(usd_matrix(target_matrix))
    marker = UsdGeom.Sphere.Define(stage, '/World/TargetTCP/Marker')
    marker.CreateRadiusAttr(0.025)
    marker.CreateDisplayColorAttr([Gf.Vec3f(0.05, 1.0, 0.15)])
    for index, color in enumerate([(1,0,0), (0,1,0), (0,0.3,1)]):
        axis = UsdGeom.BasisCurves.Define(stage, f'/World/TargetTCP/Axis{index}')
        endpoint = [0.0]*3
        endpoint[index] = 0.12
        axis.CreateTypeAttr('linear')
        axis.CreateCurveVertexCountsAttr([2])
        axis.CreatePointsAttr([Gf.Vec3f(0,0,0), Gf.Vec3f(*endpoint)])
        axis.CreateWidthsAttr([0.006, 0.006])
        axis.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    target_config = {'frame_id': 'base_link', 'tcp_link': 'tool0', 'units': 'metres',
                     'matrix_column_vectors': target_matrix.tolist(), 'calibration': 'factory' if args.physical_preview else 'generic'}
    target_rotation = usd_matrix(target_matrix).ExtractRotationQuat()
    target_config['position_xyz'] = target_matrix[:3,3].tolist()
    target_config['quaternion_xyzw'] = list(target_rotation.GetImaginary())+[target_rotation.GetReal()]
    (directory / f'{prefix}target-tcp.json').write_text(json.dumps(target_config, indent=2)+'\n')
    camera = UsdGeom.Camera.Define(stage, '/World/Camera')
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateHorizontalApertureAttr(36.0)
    camera.CreateVerticalApertureAttr(20.25)
    eye, aim = Gf.Vec3d(1.6, 1.5, 1.3), Gf.Vec3d(0, 0, 0.4)
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, aim, Gf.Vec3d(0,0,1)).GetInverse())
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    viewport = get_active_viewport()
    viewport.set_active_camera('/World/Camera')
    viewport.set_texture_resolution((1280, 720))
    start_wait = time.monotonic()
    while state['positions'] is None and time.monotonic()-start_wait < 35:
        rclpy.spin_once(node, timeout_sec=0.01)
        app.update()
    if state['positions'] is None:
        raise TimeoutError('No six-joint /joint_states. Start launch-ur-mock.sh for the offline demo.')
    preview_start = time.monotonic()
    preview_ready, preview_active = False, not args.physical_preview
    operation_kind, workflow_finished = '', False
    execution_display_error = 0.0
    planned_target_matrix = target_matrix.copy()
    status_label = None
    execute_button = None
    plan_button = None
    preview_button = None
    stop_button = None
    telemetry_label = None
    area_clear = None
    execution_id = None
    held_preview_time = 0.0

    def status(text):
        if status_label is not None:
            status_label.text = text
        print('UR_DEMO:', text, flush=True)

    def save_preview():
        (directory / f'{prefix}preview-attestation.json').write_text(json.dumps({
            'result': 'PASS', 'trajectory_sha256': trajectory_hash, 'previewed_points': len(trajectory['points']),
            'preview_duration_s': duration, 'mode': 'physical state, planning only' if args.physical_preview else 'offline mock validation',
            'operator_approved_physical': False}, indent=2)+'\n')

    def request_plan():
        global operation, operation_log, operation_kind, preview_ready, preview_active, planned_target_matrix
        if operation is not None:
            return
        matrix = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(target.GetPrim())).T
        planned_target_matrix = matrix.copy()
        config = dict(target_config, matrix_column_vectors=matrix.tolist())
        rotation = usd_matrix(matrix).ExtractRotationQuat()
        config.update(position_xyz=matrix[:3,3].tolist(),
                      quaternion_xyzw=list(rotation.GetImaginary())+[rotation.GetReal()])
        (directory / f'{prefix}target-tcp.json').write_text(json.dumps(config, indent=2)+'\n')
        environment = dict(os.environ, UR_DOD_ACTION='/move_action', UR_DOD_PIPELINE='isaac_ros_cumotion',
            UR_DOD_LIVE_STATE='1', UR_DOD_TARGET_TCP=str(directory / f'{prefix}target-tcp.json'),
            UR_DOD_TARGET_TOPIC='/ur_demo/target_tcp',
            UR_DOD_OUTPUT_PREFIX=plan_prefix, UR_DOD_URDF=str(urdf_file),
            UR_DOD_CALIBRATION=target_config['calibration'],
            UR_DOD_SEEDED_IK='1' if args.physical_preview else '0',
            UR_DOD_PREPARE_PHYSICAL='1' if args.physical_preview else '0',
            UR_DEMO_ROS_DOMAIN_ID='174' if args.physical_preview else '172')
        operation_log = (directory / f'{prefix}interactive-plan.log').open('w')
        (directory / f'{plan_prefix}-dod.json').write_text(json.dumps({'result': 'RUNNING'})+'\n')
        operation = subprocess.Popen(['/bin/bash', str(directory / 'run-host-ros.sh'), str(directory / 'test-cumotion.py')],
            env=environment, stdout=operation_log, stderr=subprocess.STDOUT, start_new_session=True)
        operation_kind, preview_ready, preview_active = 'plan', False, False
        if area_clear is not None:
            area_clear.set_value(False)
        status('Planning from current /joint_states...')

    def request_preview():
        global preview_start, preview_ready, preview_active
        if operation is not None:
            return
        if args.physical_preview and 'physical_contract' not in trajectory:
            status('Click Plan to prepare a physical trajectory first')
            return
        preview_start, preview_ready, preview_active = time.monotonic(), False, True
        status('Preview GHOST')

    def request_execute():
        global operation, operation_log, operation_kind, execution_id, held_preview_time
        if args.physical_preview:
            if not args.enable_physical_execution or operation is not None or not preview_ready:
                return
            if area_clear is None or not area_clear.get_value_as_bool():
                status('Confirm Work area is clear before Execute')
                return
            execution_id = str(uuid.uuid4())
            approval = {'execution_id': execution_id, 'trajectory_sha256': trajectory_hash,
                'operator_approved_physical': True, 'approved_unix_s': time.time(), 'work_area_clear': True,
                'source': 'Isaac Sim Execute on UR5e button', 'target_tcp': planned_target_matrix.tolist()}
            approval_path = directory / 'physical-execution-approval.json'
            approval_path.write_text(json.dumps(approval, indent=2)+'\n')
            (directory / 'physical-execute-dod.json').write_text(json.dumps({
                'result': 'RUNNING', 'execution_id': execution_id, 'physical_trajectory_sent': False})+'\n')
            operation_log = (directory / 'physical-interactive-execute.log').open('w')
            operation = subprocess.Popen(['/bin/bash', str(directory / 'run-host-ros.sh'),
                str(directory / 'execute-physical-plan.py'), '--execute', '--trajectory', str(trajectory_path),
                '--approval', str(approval_path)],
                env=dict(os.environ, UR_DEMO_ROS_DOMAIN_ID='174'),
                stdout=operation_log, stderr=subprocess.STDOUT, start_new_session=True)
            operation_kind, held_preview_time = 'execute', 0.0
            status('Execute on UR5e - checking robot and arming controller...')
            return
        if operation is not None or not preview_ready:
            return
        (directory / 'execution-state.json').write_text(json.dumps({
            'state': 'QUEUED', 'trajectory_sha256': trajectory_hash})+'\n')
        operation_log = (directory / 'interactive-execute.log').open('w')
        operation = subprocess.Popen(['/bin/bash', str(directory / 'run-host-ros.sh'),
            str(directory / 'execute-mock-plan.py'), '--trajectory', str(trajectory_path)],
            stdout=operation_log, stderr=subprocess.STDOUT, start_new_session=True)
        operation_kind = 'execute'
        status('Execute MOCK - REAL follows the controller')

    def request_stop():
        if args.physical_preview and operation_kind == 'execute' and operation is not None:
            path = directory / 'physical-stop-request.json'
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'execution_id': execution_id})+'\n')
            temporary.replace(path)
            status('Stop requested - canceling trajectory and deactivating controller...')

    def request_reset_tcp():
        global preview_ready, preview_active
        if operation is not None:
            return
        matrix = kinematics.world_transforms(state['positions'])['tool0']
        # Gizmo editing can replace the original transform operation stack.
        target.ClearXformOpOrder()
        target.MakeMatrixXform().Set(usd_matrix(matrix))
        preview_ready, preview_active = False, False
        status('TargetTCP reset to current REAL TCP - move target and click Plan')

    if not args.headless:
        import omni.ui as ui
        window = ui.Window('UR5e - Plan / Preview / Execute' if args.enable_physical_execution else 'UR5e - REAL / Plan / Preview' if args.physical_preview else 'UR5e - Plan / Preview / Execute MOCK', width=560, height=365)
        with window.frame:
            with ui.VStack(spacing=8):
                ui.Label('UR5e 192.168.60.201 - factory calibration - TCP 0' if args.physical_preview else 'Offline - UR5e GenericSystem mock', height=25)
                ui.Label('TargetTCP: /World/TargetTCP - base_link -> tool0', height=25)
                ui.Label('Move or rotate TargetTCP in Stage, then click Plan.', height=25)
                with ui.HStack(height=35, spacing=6):
                    plan_button = ui.Button('Plan', clicked_fn=request_plan)
                    preview_button = ui.Button('Preview', clicked_fn=request_preview)
                    execute_button = ui.Button('Execute on UR5e' if args.enable_physical_execution else 'Execute - disabled' if args.physical_preview else 'Execute MOCK', clicked_fn=request_execute, enabled=False)
                if args.physical_preview:
                    ui.Button('Reset TargetTCP to current REAL TCP', clicked_fn=request_reset_tcp, height=30)
                if args.enable_physical_execution:
                    with ui.HStack(height=25):
                        area_clear = ui.SimpleBoolModel(False)
                        ui.CheckBox(model=area_clear, width=25)
                        ui.Label('Work area is clear (surroundings are not modeled)')
                    stop_button = ui.Button('Stop', clicked_fn=request_stop, height=35, enabled=False)
                    telemetry_label = ui.Label('External Control status / speed scaling', height=25)
                status_label = ui.Label('REAL follows the robot - move TargetTCP and click Plan' if args.physical_preview else 'Initial trajectory preview', word_wrap=True)
    if args.plan_preview_test:
        expected_reset = kinematics.world_transforms(state['positions'])['tool0']
        target.AddTranslateOp(opSuffix='resetTest').Set(Gf.Vec3d(0.03, 0, 0))
        request_reset_tcp()
        reset_matrix = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(target.GetPrim())).T
        report['reset_target_tcp_max_error'] = float(np.max(np.abs(reset_matrix-expected_reset)))
        if report['reset_target_tcp_max_error'] > 1e-9:
            raise RuntimeError('Reset TargetTCP did not restore actual tool0 pose')
        matrix = kinematics.world_transforms(state['positions'])['tool0']
        matrix[2,3] += 0.01
        target.MakeMatrixXform().Set(usd_matrix(matrix))
        # Publish this pose before the planning client starts waiting for the live topic.
        app.update()
    if args.workflow_test or args.plan_preview_test:
        request_plan()
    frames, max_fk_error, real_max_fk_error = 0, 0.0, 0.0
    while app.is_running() and not shutdown_requested:
        rclpy.spin_once(node, timeout_sec=0.0)
        if time.monotonic()-state['received'] > 0.5:
            raise RuntimeError('REAL state stale for more than 0.5 seconds')
        if operation is not None and operation.poll() is not None:
            finished_kind, exit_code = operation_kind, operation.returncode
            operation, operation_kind = None, ''
            operation_log.close()
            operation_log = None
            dod_path = directory / (f'{plan_prefix}-dod.json' if finished_kind == 'plan' else 'physical-execute-dod.json' if args.physical_preview else 'mock-execute-dod.json')
            result = json.loads(dod_path.read_text()) if dod_path.exists() else {'result': 'FAIL'}
            if finished_kind == 'execute' and args.physical_preview:
                report['physical_trajectory_sent'] |= result.get('physical_trajectory_sent', False)
                report['last_execution'] = result
            if exit_code != 0 or result.get('result') != 'PASS':
                status(f"{finished_kind}: {result.get('result', 'FAIL')} - {result.get('error', 'check log')}")
                if finished_kind == 'execute':
                    preview_ready, preview_active = False, False
                if args.workflow_test or args.plan_preview_test:
                    raise RuntimeError(f'{finished_kind} failed: {result}')
            elif finished_kind == 'plan':
                trajectory_path = directory / f'ur5e-{plan_prefix}-planned-trajectory.json'
                trajectory_bytes = trajectory_path.read_bytes()
                trajectory, trajectory_hash = json.loads(trajectory_bytes), hashlib.sha256(trajectory_bytes).hexdigest()
                if trajectory['joint_names'] != names:
                    raise ValueError('Live plan joint order changed')
                duration = trajectory['points'][-1]['time_from_start_s']
                held_preview_time = 0.0
                planned_target_matrix = np.asarray(trajectory['target_tcp']['matrix_column_vectors'])
                preview_start, preview_ready, preview_active = time.monotonic(), False, True
                status(f'Plan ready - {duration:.2f} s - GHOST preview')
            else:
                preview_ready, preview_active = False, False
                status('Execute on UR5e: PASS - planned-vs-actual saved; Plan again' if args.physical_preview else 'Execute MOCK: PASS - planned-vs-actual saved')
                workflow_finished = True
        elapsed = time.monotonic()-preview_start
        preview_time = min(elapsed, duration) if preview_active or not args.physical_preview else held_preview_time
        execution_running = operation_kind == 'execute'
        if execution_running:
            preview_time = 0.0
            execution_path = directory / ('physical-execution-state.json' if args.physical_preview else 'execution-state.json')
            if execution_path.exists():
                try:
                    running = json.loads(execution_path.read_text())
                    if args.physical_preview and running.get('execution_id') == execution_id and running.get('trajectory_sha256') == trajectory_hash:
                        preview_time = max(0.0, min(running.get('trajectory_time_s', 0.0), duration))
                        held_preview_time = preview_time
                    elif running.get('state') == 'RUNNING' and running.get('trajectory_sha256') == trajectory_hash:
                        preview_time = max(0.0, min(time.time()-running['start_unix_s'], duration))
                    elif running.get('state') == 'DONE' and running.get('trajectory_sha256') == trajectory_hash:
                        preview_time = duration
                except json.JSONDecodeError:
                    pass
        ghost_positions = model.sample_trajectory(trajectory, preview_time)
        if execution_running:
            execution_display_error = max(execution_display_error,
                float(np.max(np.abs(ghost_positions-np.asarray([state['positions'][n] for n in names])))))
        for twin, positions in [('REAL', state['positions']), ('GHOST', dict(zip(names, ghost_positions)))]:
            local = kinematics.local_transforms(positions)
            for link, op in twin_maps[twin].items():
                op.Set(usd_matrix(local.get(link, np.eye(4))))
            computed = UsdGeom.XformCache().GetLocalToWorldTransform(twin_maps[twin]['tool0'].GetAttr().GetPrim())
            expected = kinematics.world_transforms(positions)['tool0']
            error = float(np.max(np.abs(np.asarray(computed).T-expected)))
            if twin == 'GHOST':
                max_fk_error = max(max_fk_error, error)
            else:
                real_max_fk_error = max(real_max_fk_error, error)
        mirror = JointState()
        mirror.header.stamp = node.get_clock().now().to_msg()
        mirror.name, mirror.position = names, [state['positions'][n] for n in names]
        mirror_publisher.publish(mirror)
        # Editing /World/TargetTCP in the GUI changes the published target pose.
        target_world = UsdGeom.XformCache().GetLocalToWorldTransform(target.GetPrim())
        translation = target_world.ExtractTranslation()
        quaternion = target_world.ExtractRotationQuat()
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = translation
        pose.pose.orientation.w = quaternion.GetReal()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = quaternion.GetImaginary()
        target_publisher.publish(pose)
        current_target_matrix = np.asarray(target_world).T
        target_changed = float(np.max(np.abs(current_target_matrix-planned_target_matrix))) > 1e-6
        if target_changed:
            preview_ready = False
            if area_clear is not None:
                area_clear.set_value(False)
        if preview_active and elapsed >= duration and not target_changed:
            preview_ready, preview_active = True, False
            held_preview_time = duration
            save_preview()
            status('Preview complete - confirm clear work area, then Execute on UR5e' if args.enable_physical_execution else 'Preview complete - physical execution disabled' if args.physical_preview else 'Preview complete - Execute MOCK is available')
            if args.workflow_test:
                request_execute()
            if args.plan_preview_test:
                if args.enable_physical_execution:
                    check = subprocess.run(['/bin/bash', str(directory / 'run-host-ros.sh'),
                        str(directory / 'execute-physical-plan.py'), '--check-only', '--trajectory', str(trajectory_path)],
                        env=dict(os.environ, UR_DEMO_ROS_DOMAIN_ID='174'), capture_output=True, text=True, timeout=40)
                    (directory / 'physical-preflight-test.log').write_text(check.stdout+check.stderr)
                    report['execution_preflight'] = json.loads((directory / 'physical-preflight-dod.json').read_text())
                    if check.returncode or report['execution_preflight']['result'] != 'PASS':
                        raise RuntimeError('Read-only Execute preflight failed: '+check.stdout[-1500:])
                workflow_finished = True
        if execute_button is not None:
            at_start = max(abs(state['positions'][n]-trajectory['points'][0]['positions'][i]) for i,n in enumerate(names)) < .003
            physical_ready = args.enable_physical_execution and 'physical_contract' in trajectory and at_start and robot_status['program'] and area_clear.get_value_as_bool()
            execute_button.enabled = preview_ready and operation is None and (physical_ready if args.physical_preview else True)
            plan_button.enabled = operation is None
            preview_button.enabled = operation is None and not target_changed
        if stop_button is not None:
            stop_button.enabled = operation_kind == 'execute' and operation is not None
            tracking = ''
            if execution_running and 'running' in globals() and running.get('execution_id') == execution_id:
                tracking = f" | Error: {running.get('max_tracking_error_rad', 0):.4f} rad"
            telemetry_label.text = f"External Control: {'RUNNING' if robot_status['program'] else 'STOPPED'} | Speed: {robot_status['speed']*100:.0f}% | GHOST: {preview_time:.2f}/{duration:.2f} s"+tracking
        app.update()
        frames += 1
        if (args.workflow_test or args.plan_preview_test) and workflow_finished:
            break
        if args.test and not args.workflow_test and elapsed >= duration:
            break
        if (args.workflow_test or args.plan_preview_test) and time.monotonic()-start_wait > 120:
            raise TimeoutError('Interactive workflow timeout')
    if max_fk_error > 1e-6 or real_max_fk_error > 1e-6:
        raise RuntimeError(f'USD/FK disagreement: ghost={max_fk_error}, REAL={real_max_fk_error}')
    stage.GetRootLayer().Export(str(directory / f'{prefix}ur5e-twins.usda'))
    if args.test or args.workflow_test or args.plan_preview_test:
        capture = capture_viewport_to_file(viewport, str(directory / f'{prefix}ur5e-preview.png'))
        for _ in range(30):
            app.update()
    report.update(result='PASS', rendered_frames=frames, valid_state_samples=state['samples'],
        trajectory_sha256=trajectory_hash, trajectory_file=str(trajectory_path),
        trajectory_points=len(trajectory['points']), preview_duration_s=duration,
        ghost_tcp_fk_max_error=max_fk_error, real_tcp_fk_max_error=real_max_fk_error,
        ros_state_age_p95_s=float(np.percentile(state['ages'],95)) if state['ages'] else None,
        target_frame='base_link', tcp='tool0', source_domain=node.context.get_domain_id())
    if args.workflow_test:
        report.update(workflow='live TargetTCP -> MoveIt/cuMotion -> GHOST preview -> FJT mock -> REAL',
                      execution_display_max_joint_error_rad=execution_display_error)
        if execution_display_error > 0.03:
            report['result'] = 'FAIL'
            raise RuntimeError(f'Execution display error: {execution_display_error} rad')
    if args.plan_preview_test:
        report.update(workflow='physical /joint_states -> TargetTCP -> MoveIt/cuMotion -> GHOST preview',
                      plan_preview_completed=workflow_finished)
    print('TWINS_DOD:', json.dumps(report))
except Exception as exc:
    report.update(result='FAIL', error=str(exc))
    raise
finally:
    (directory / f'{prefix}twins-dod.json').write_text(json.dumps(report, indent=2)+'\n')
    if operation is not None and operation.poll() is None:
        if args.physical_preview and operation_kind == 'execute':
            request_stop()
            os.killpg(operation.pid, signal.SIGTERM)
            # The host executor cancels its goal and deactivates its controller.
            # Never SIGKILL it while a physical trajectory might still be owned.
            operation.wait()
        else:
            os.killpg(operation.pid, signal.SIGTERM)
            try:
                operation.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(operation.pid, signal.SIGKILL)
                operation.wait()
    if operation_log is not None:
        operation_log.close()
    if node is not None:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    app.close()
