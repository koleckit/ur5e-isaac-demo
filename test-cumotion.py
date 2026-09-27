"""Plan an offline UR5e trajectory through the cuMotion MoveGroup backend."""
import json
import importlib.util
import os
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from action_msgs.msg import GoalStatus

names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
start = [0.0, -2.2, 1.9, -1.383, -1.57, 0.0]
target = list(start)
target[0] += 0.15
action_name = os.environ.get('UR_DOD_ACTION', '/cumotion/move_group')
output_prefix = os.environ.get('UR_DOD_OUTPUT_PREFIX', 'cumotion')
report = {'result': 'FAIL', 'test': 'cuMotion plan only', 'action': action_name, 'joint_names': names}
source = Path(__file__).resolve().parent
output = Path(os.environ.get('UR_DOD_OUTPUT_DIR', str(source))).resolve()
output.mkdir(parents=True, exist_ok=True)
course = None
urdf_file = Path(os.environ.get('UR_DOD_URDF', str(source / 'ur5e.urdf')))
calibration_label = os.environ.get('UR_DOD_CALIBRATION', 'generic')
report.update(urdf=str(urdf_file), calibration=calibration_label)
target_tcp_file = os.environ.get('UR_DOD_TARGET_TCP')
target_topic = os.environ.get('UR_DOD_TARGET_TOPIC')
rclpy.init()
node = rclpy.create_node('ur5e_cumotion_dod')
received_target = {'message': None, 'received': 0.0}
if target_topic:
    def on_target(message):
        received_target.update(message=message, received=time.monotonic())
    target_subscription = node.create_subscription(PoseStamped, target_topic, on_target, 10)
live_state = {'received': 0.0}
if os.environ.get('UR_DOD_LIVE_STATE') == '1':
    from rclpy.qos import qos_profile_sensor_data
    def on_state(message):
        mapped = dict(zip(message.name, message.position))
        if all(n in mapped for n in names):
            start[:] = [mapped[n] for n in names]
            live_state['received'] = time.monotonic()
    subscription = node.create_subscription(JointState, '/joint_states', on_state, qos_profile_sensor_data)
    publisher = None
else:
    publisher = node.create_publisher(JointState, '/joint_states', 10)

def publish_state():
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = names
    msg.position = start
    msg.velocity = [0.0] * 6
    publisher.publish(msg)

timer = node.create_timer(0.02, publish_state) if publisher is not None else None
client = ActionClient(node, MoveGroup, action_name)

def wait_until(predicate, timeout, reason):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError(reason)
        rclpy.spin_once(node, timeout_sec=0.1)

try:
    wait_until(client.server_is_ready, 150, 'cuMotion action server did not become ready')
    if publisher is None:
        wait_until(lambda: live_state['received'] > 0, 15, 'Missing live joint state')
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher is None and time.monotonic()-live_state['received'] > 0.2:
        raise RuntimeError('Live planning state stale')
    if target_topic:
        wait_until(lambda: received_target['message'] is not None, 15, 'Missing TargetTCP PoseStamped')
        if time.monotonic()-received_target['received'] > 0.2:
            raise RuntimeError('TargetTCP pose stale')
    if os.environ.get('UR_DOD_JOINT_GOAL'):
        target = json.loads(os.environ['UR_DOD_JOINT_GOAL'])
        if len(target) != 6 or not all(isinstance(q,(int,float)) for q in target):
            raise ValueError('Invalid qualification joint goal')
    elif not target_tcp_file and not target_topic:
        target = list(start)
        target[0] += 0.15
    if os.environ.get('UR_DOD_COURSE_CONFIG'):
        import sys
        sys.path.insert(0, str(source / 'obstacle_course/src'))
        from world import load
        from ros_scene import Scene, RuntimeProof
        course = load(os.environ['UR_DOD_COURSE_CONFIG'], os.environ['UR_DOD_COURSE_SCENARIO'])
        course_scene = Scene(node, wait_until)
        course_enabled = os.environ.get('UR_DOD_COURSE_ENABLED', '1') == '1'
        course_scene.apply(course, enabled=course_enabled)
        proof = RuntimeProof(node, course_scene)
        proof.begin()
    goal = MoveGroup.Goal()
    goal.request.group_name = 'ur_manipulator'
    goal.request.planner_id = 'cuMotion'
    if 'UR_DOD_PIPELINE' in os.environ:
        goal.request.pipeline_id = os.environ['UR_DOD_PIPELINE']
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = 0.2
    goal.request.max_acceleration_scaling_factor = 0.2
    goal.request.start_state.joint_state.name = names
    goal.request.start_state.joint_state.position = start
    goal.request.start_state.joint_state.velocity = [0.0] * 6
    goal.request.start_state.is_diff = False
    constraints = Constraints(name='offline_ur5e_joint_goal')
    if target_tcp_file or target_topic:
        import numpy as np
        if target_topic:
            message = received_target['message']
            q = message.pose.orientation
            x, y, z, w = q.x, q.y, q.z, q.w
            if not np.isclose(x*x+y*y+z*z+w*w, 1.0, atol=1e-5):
                raise ValueError('TargetTCP quaternion is not normalized')
            pose_matrix = np.eye(4)
            pose_matrix[:3,:3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
            p = message.pose.position
            pose_matrix[:3,3] = [p.x,p.y,p.z]
            target_config = {'frame_id': message.header.frame_id, 'tcp_link': 'tool0', 'units': 'metres',
                'matrix_column_vectors': pose_matrix.tolist(), 'pose_topic': target_topic, 'calibration': calibration_label}
            report['target_source'] = target_topic
        else:
            target_config = json.loads(Path(target_tcp_file).read_text())
        matrix = np.asarray(target_config['matrix_column_vectors'])
        if target_config['frame_id'] != 'base_link' or target_config['tcp_link'] != 'tool0':
            raise ValueError('TCP test expects base_link → tool0')
        # Matrix to quaternion (xyzw), including rotations whose trace is negative.
        rotation = matrix[:3,:3]
        eigen_matrix = np.array([
            [rotation[0,0]-rotation[1,1]-rotation[2,2], rotation[1,0]+rotation[0,1], rotation[2,0]+rotation[0,2], rotation[2,1]-rotation[1,2]],
            [rotation[1,0]+rotation[0,1], rotation[1,1]-rotation[0,0]-rotation[2,2], rotation[2,1]+rotation[1,2], rotation[0,2]-rotation[2,0]],
            [rotation[2,0]+rotation[0,2], rotation[2,1]+rotation[1,2], rotation[2,2]-rotation[0,0]-rotation[1,1], rotation[1,0]-rotation[0,1]],
            [rotation[2,1]-rotation[1,2], rotation[0,2]-rotation[2,0], rotation[1,0]-rotation[0,1], np.trace(rotation)]])/3
        eigenvalues, eigenvectors = np.linalg.eigh(eigen_matrix)
        quaternion = eigenvectors[:,np.argmax(eigenvalues)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = matrix[:3,3].tolist()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion.tolist()
        position = PositionConstraint()
        position.header.frame_id, position.link_name, position.weight = 'base_link', 'tool0', 1.0
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        center = Pose()
        center.position = pose.position
        center.orientation.w = 1.0
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [center]
        orientation = OrientationConstraint()
        orientation.header.frame_id, orientation.link_name, orientation.weight = 'base_link', 'tool0', 1.0
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = orientation.absolute_z_axis_tolerance = 0.005
        constraints.position_constraints, constraints.orientation_constraints = [position], [orientation]
        report['goal_mode'] = 'TargetTCP pose in base_link, tool0'
        if os.environ.get('UR_DOD_SEEDED_IK') == '1':
            # Select an IK branch using the current robot configuration before GPU planning.
            # Task-space IK in cuMotion can otherwise select a distant wrist configuration.
            ik_client = node.create_client(GetPositionIK, '/compute_ik')
            if not ik_client.wait_for_service(timeout_sec=10):
                raise TimeoutError('MoveIt seeded IK service unavailable')
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'ur_manipulator'
            request.ik_request.ik_link_name = 'tool0'
            request.ik_request.robot_state = goal.request.start_state
            request.ik_request.pose_stamped.header.frame_id = 'base_link'
            request.ik_request.pose_stamped.pose = pose
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 2
            if course:
                from seeded_ik import solve
                solutions, report['course_ik'] = solve(ik_client,request,wait_until,names,start,urdf_file)
                solution = solutions[0]
            else:
                ik_future = ik_client.call_async(request)
                wait_until(ik_future.done, 10, 'Seeded IK response timeout')
                ik = ik_future.result()
                if ik.error_code.val != MoveItErrorCodes.SUCCESS:
                    raise RuntimeError(f'No collision-free seeded IK solution: {ik.error_code.val}')
                solution = dict(zip(ik.solution.joint_state.name, ik.solution.joint_state.position))
            target = [solution[name] for name in names]
            constraints.position_constraints = []
            constraints.orientation_constraints = []
            constraints.joint_constraints = [JointConstraint(joint_name=name, position=value,
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
                for name, value in zip(names, target)]
            report.update(goal_mode='TargetTCP -> seeded MoveIt IK -> cuMotion joint goal',
                          ik_seed=list(goal.request.start_state.joint_state.position),
                          ik_solution=target,
                          ik_max_joint_delta_rad=max(abs(a-b) for a,b in zip(target, start)))
    else:
        constraints.joint_constraints = [JointConstraint(joint_name=name, position=q,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for name, q in zip(names, target)]
    candidate_goals = [[s[name] for name in names] for s in solutions] if course and os.environ.get('UR_DOD_SEEDED_IK') == '1' else [target]
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = True
    goal.planning_options.planning_scene_diff.is_diff = True
    attempts=[]
    for candidate in candidate_goals:
        if course and os.environ.get('UR_DOD_SEEDED_IK') == '1':
            goal.request.goal_constraints[0].joint_constraints = [JointConstraint(joint_name=name,
                position=value,tolerance_above=.001,tolerance_below=.001,weight=1.)
                for name,value in zip(names,candidate)]
        if course:
            proof.arm()
        submitted=client.send_goal_async(goal)
        wait_until(submitted.done,15,'No planning acceptance response')
        handle=submitted.result()
        if not handle.accepted:
            raise RuntimeError('cuMotion rejected plan request')
        response=handle.get_result_async()
        wait_until(response.done,90,'cuMotion did not finish planning')
        result=response.result()
        trajectory=result.result.planned_trajectory.joint_trajectory
        attempt={'joint_goal':candidate,'action_status':result.status,
            'moveit_error_code':result.result.error_code.val,
            'planning_time_s':result.result.planning_time,'point_count':len(trajectory.points)}
        if course:
            if result.result.error_code.val==MoveItErrorCodes.SUCCESS:
                attempt['runtime_planner']=proof.finish(wait_until)
            else:
                attempt['runtime_planner']=dict(proof.proof,backend_goals=proof.new_goals())
        attempts.append(attempt)
        if result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code.val==MoveItErrorCodes.SUCCESS:
            target=candidate
            report.update(action_status=result.status,moveit_error_code=result.result.error_code.val,
                planning_time_s=result.result.planning_time,point_count=len(trajectory.points))
            if course:
                report['runtime_planner']=attempt['runtime_planner']
                report['course_planning_attempts']=attempts
                report['ik_solution']=target
                report['ik_max_joint_delta_rad']=max(abs(a-b) for a,b in zip(target,start))
            break
    else:
        report['course_planning_attempts']=attempts
        report.update(action_status=result.status,moveit_error_code=result.result.error_code.val,
            planning_time_s=result.result.planning_time,point_count=len(trajectory.points))
        if course:
            report['runtime_planner']=attempts[-1]['runtime_planner']
        raise RuntimeError(f'cuMotion plan failed after {len(attempts)} IK candidates')
    if len(trajectory.points) < 2 or set(trajectory.joint_names) != set(names):
        raise RuntimeError('Invalid or empty planned trajectory')
    indices = [trajectory.joint_names.index(name) for name in names]
    end = [trajectory.points[-1].positions[i] for i in indices]
    report.update(endpoint_joint_positions=end,
                  trajectory_times_s=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points])
    if os.environ.get('UR_DOD_SEEDED_IK') == '1':
        report['endpoint_ik_max_joint_error_rad'] = max(abs(a-b) for a,b in zip(end, target))
    if target_tcp_file or target_topic:
        spec = importlib.util.spec_from_file_location('demo_model', source / 'demo-model.py')
        model = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(model)
        actual_tcp = model.Kinematics(urdf_file).world_transforms(dict(zip(names,end)))['tool0']
        error = float(np.linalg.norm(actual_tcp[:3,3]-matrix[:3,3]))
        rotation_error = float(np.arccos(np.clip((np.trace(actual_tcp[:3,:3].T@matrix[:3,:3])-1)/2, -1, 1)))
        report.update(endpoint_position_error_m=error, endpoint_orientation_error_rad=rotation_error)
        if error > 0.002 or rotation_error > 0.01:
            report['rejected_trajectory_points'] = [{'positions': list(p.positions),
                'velocities': list(p.velocities)} for p in trajectory.points]
            raise RuntimeError(f'TCP goal mismatch: {error} m / {rotation_error} rad')
    else:
        error = max(abs(a-b) for a,b in zip(end, target))
        report['endpoint_max_error_rad'] = error
        if error > 0.01:
            raise RuntimeError(f'Planned endpoint differs from target: {error}')
    planned = {'joint_names': list(trajectory.joint_names), 'points': [{
        'positions': list(p.positions), 'velocities': list(p.velocities),
        'accelerations': list(p.accelerations),
        'time_from_start_s': p.time_from_start.sec + p.time_from_start.nanosec * 1e-9}
        for p in trajectory.points]}
    if target_tcp_file or target_topic:
        planned['target_tcp'] = target_config
    times = [p['time_from_start_s'] for p in planned['points']]
    if any(b <= a for a,b in zip(times, times[1:])):
        raise RuntimeError('Trajectory timestamps do not increase strictly')
    if os.environ.get('UR_DOD_PREPARE_PHYSICAL') == '1' or course:
        import yaml
        from moveit_msgs.srv import GetStateValidity, GetPlanningScene
        def load_module(name, filename):
            module_spec = importlib.util.spec_from_file_location(name, source / filename)
            module = importlib.util.module_from_spec(module_spec)
            module_spec.loader.exec_module(module)
            return module
        contract = load_module('execution_contract', 'execution-contract.py')
        scene_tools = load_module('scene_fingerprint', 'scene-fingerprint.py')
        calibration_hash = (yaml.safe_load((urdf_file.parent / 'ur5e-calibration.yaml').read_text())['kinematics']['hash']
            if os.environ.get('UR_DOD_PREPARE_PHYSICAL') == '1' else 'generic_obstacle_course')
        planned = contract.prepare(planned, urdf_file, calibration_hash)
        scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        validity_client = node.create_client(GetStateValidity, '/check_state_validity')
        for service in (scene_client, validity_client):
            if not service.wait_for_service(timeout_sec=10):
                raise TimeoutError('Physical planning scene service unavailable')
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = scene_tools.COMPONENTS
        scene_future = scene_client.call_async(scene_request)
        wait_until(scene_future.done, 10, 'Planning scene snapshot timeout')
        scene = scene_future.result().scene
        scene_hash = scene_tools.fingerprint(scene)
        sample_times = contract.collision_sample_times(planned)
        model = load_module('demo_model', 'demo-model.py')
        for seconds in sample_times:
            request = GetStateValidity.Request()
            request.group_name = 'ur_manipulator'
            request.robot_state.joint_state.name = names
            request.robot_state.joint_state.position = model.sample_trajectory(planned, seconds).tolist()
            validity = validity_client.call_async(request)
            wait_until(validity.done, 5, 'Collision validation timeout')
            if not validity.result().valid:
                raise RuntimeError(f'Interpolated physical trajectory collision at {seconds:.3f} s')
        final_scene = scene_client.call_async(scene_request)
        wait_until(final_scene.done, 10, 'Final scene snapshot timeout')
        if scene_tools.fingerprint(final_scene.result().scene) != scene_hash:
            raise RuntimeError('Collision scene changed while validating the plan')
        planned['physical_contract'].update(scene_sha256=scene_hash,
            collision_validation='MoveIt state validity on interpolated spline samples',
            collision_samples=len(sample_times), world_collision_objects=len(scene.world.collision_objects))
        report.update(physical_contract=planned['physical_contract'],
                      physical_trajectory_prepared=True)
        times = [p['time_from_start_s'] for p in planned['points']]
    if course:
        course_scene.verify(course, enabled=course_enabled)
        current = load(course['config_path'], course['scenario'])
        if current['geometry_sha256'] != course['geometry_sha256']:
            raise RuntimeError('Obstacle configuration changed while planning')
        planned['obstacle_course'] = dict(course, enabled=course_enabled, runtime_planner=report['runtime_planner'])
        report.update(obstacle_course=course['scenario'], geometry_sha256=course['geometry_sha256'],
            world_collision_objects=len(course['obstacles']) if course_enabled else 0,
            start_pose=model.Kinematics(urdf_file).world_transforms(dict(zip(names,start)))['tool0'].tolist(),
            target_pose=target_config['matrix_column_vectors'], collision_free=True)
    (output / f'ur5e-{output_prefix}-planned-trajectory.json').write_text(json.dumps(planned, indent=2)+'\n')
    report.update(result='PASS', duration_s=times[-1])
    print(json.dumps(report, indent=2))
except Exception as exc:
    report['error'] = str(exc)
    report['planning_status'] = 'PLANNING FAILED'
    (output / f'ur5e-{output_prefix}-planned-trajectory.json').unlink(missing_ok=True)
    raise
finally:
    (output / f'{output_prefix}-dod.json').write_text(json.dumps(report, indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
