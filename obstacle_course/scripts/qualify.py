"""Actual GPU/FCL/Isaac/FJT qualification on an isolated GenericSystem domain."""
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path
import numpy as np
import rclpy
from moveit_msgs.srv import GetStateValidity

root = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'obstacle_course/src'))
from world import load, signed_box_distance
from ros_scene import Scene
def module(name, filename):
    spec=importlib.util.spec_from_file_location(name,root/filename)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
model=module('model','demo-model.py'); contract=module('contract','execution-contract.py')
kinematics=model.Kinematics(root/'ur5e.urdf')
config=root/'obstacle_course/config/obstacle_course.yaml'
out_root=root/'obstacle_course/runs'
if os.environ.get('ROS_DOMAIN_ID') != '178':
    raise RuntimeError('Qualification must use isolated mock domain 178')
report={'result':'FAIL','physical_trajectory_sent':False,'scenarios':{}}
all_scenarios=('single_wall','corridor','gate_3d')
negative_only=os.environ.get('UR_COURSE_NEGATIVE_ONLY')=='1'
selected=() if negative_only else tuple(os.environ.get('UR_COURSE_SCENARIOS',','.join(all_scenarios)).split(','))
if (not selected and not negative_only) or any(s not in all_scenarios for s in selected):
    raise ValueError('Unknown qualification scenario')
rclpy.init();node=rclpy.create_node('obstacle_course_qualification')
def wait(predicate, timeout, reason):
    until=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>until:raise TimeoutError(reason)
        rclpy.spin_once(node,timeout_sec=.05)
scene=Scene(node,wait)
def command(argv,out,name,env=None,timeout=240):
    with (out/(name+'.log')).open('w') as log:
        result=subprocess.run(argv,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
    if result.returncode:raise RuntimeError(name+' failed: '+str(out/(name+'.log')))
def inspect(plan,course):
    collisions=[];tcp_only_misses=[]
    for seconds in contract.collision_sample_times(plan):
        q=model.sample_trajectory(plan,seconds)
        req=GetStateValidity.Request();req.group_name='ur_manipulator'
        req.robot_state.joint_state.name=plan['joint_names'];req.robot_state.joint_state.position=q.tolist()
        result=scene.rpc(GetStateValidity,'/check_state_validity',req)
        if not result.valid:
            tcp=kinematics.world_transforms(dict(zip(plan['joint_names'],q)))['tool0'][:3,3]
            item={'time_s':seconds,'contacts':[{'body1':c.contact_body_1,'body2':c.contact_body_2} for c in result.contacts],
                'tcp_clearance_m':min(signed_box_distance(tcp,b) for b in course['obstacles'])}
            collisions.append(item)
            if item['tcp_clearance_m']>0:tcp_only_misses.append(item)
    return collisions,tcp_only_misses
try:
    # Wait for both servers; no fake planner, no perception pipeline.
    from rclpy.action import ActionClient
    from moveit_msgs.action import MoveGroup
    clients=[ActionClient(node,MoveGroup,a) for a in ('/move_action','/cumotion/move_group')]
    wait(lambda:all(c.server_is_ready() for c in clients),150,'GPU planner startup timeout')
    for name in selected:
        out=out_root/name;out.mkdir(parents=True,exist_ok=True)
        command(['/usr/bin/python3',str(root/'obstacle_course/scripts/prepare.py'),'--scenario',name,'--output-dir',str(out)],out,'inputs')
        course=load(config,name)
        scene.apply(course,enabled=False)
        command(['/usr/bin/python3',str(root/'execute-mock-plan.py'),'--prepare','--trajectory',str(out/'seed.json'),'--output-dir',str(out)],out,'reset-mock')
        environment=dict(os.environ,UR_DOD_ACTION='/move_action',UR_DOD_PIPELINE='isaac_ros_cumotion',UR_DOD_LIVE_STATE='1',
            UR_DOD_TARGET_TCP=str(out/'target.json'),UR_DOD_SEEDED_IK='1',UR_DOD_OUTPUT_PREFIX='baseline',
            UR_DOD_OUTPUT_DIR=str(out),UR_DOD_COURSE_CONFIG=str(config),UR_DOD_COURSE_SCENARIO=name,UR_DOD_COURSE_ENABLED='0')
        command(['/usr/bin/python3',str(root/'test-cumotion.py')],out,'baseline-plan',environment)
        baseline=json.loads((out/'ur5e-baseline-planned-trajectory.json').read_text())
        scene.apply(course)
        blocked,tcp_misses=inspect(baseline,course)
        if not blocked:raise RuntimeError(name+': baseline is not blocked; scene does not demonstrate a detour')
        if name=='gate_3d' and not tcp_misses:
            raise RuntimeError('Gate must demonstrate robot collision while the point TCP is clear')
        command(['/bin/bash',str(root/'run-isaac-python.sh'),str(root/'isaac-twins.py'),'--headless','--course-test',
            '--obstacle-course',str(config),'--scenario',name,'--output-dir',str(out),'--trajectory',str(out/'seed.json')],out,'isaac')
        preview=json.loads((out/'twins-dod.json').read_text())
        if preview['result']!='PASS':raise RuntimeError('Isaac preview failed: '+preview.get('error','unknown'))
        plan_file=out/'ur5e-interactive-cumotion-planned-trajectory.json';plan=json.loads(plan_file.read_text())
        remaining,_=inspect(plan,course)
        if remaining:raise RuntimeError(name+': final prepared trajectory collides')
        command(['/usr/bin/python3',str(root/'execute-mock-plan.py'),'--trajectory',str(plan_file),'--output-dir',str(out)],out,'execute-mock',timeout=180)
        metrics=json.loads((out/'interactive-cumotion-dod.json').read_text())
        execution=json.loads((out/'mock-execute-dod.json').read_text())
        if preview['result']!='PASS' or not preview.get('course_preview_completed') or execution['result']!='PASS':
            raise RuntimeError('Preview/execution validation failed')
        report['scenarios'][name]={'result':'PASS','planning_time_s':metrics['planning_time_s'],
            'trajectory_duration_s':metrics['duration_s'],'points':metrics['point_count'],
            'collision_objects':metrics['world_collision_objects'],'runtime_planner':metrics['runtime_planner'],
            'geometry_sha256':course['geometry_sha256'],'baseline_collision_samples':len(blocked),
            'tcp_only_missed_collision_samples':len(tcp_misses),'baseline_contacts':blocked[:8],
            'collision_samples':plan['physical_contract']['collision_samples'],
            'preview_sha256':hashlib.sha256(plan_file.read_bytes()).hexdigest(),
            'isaac_geometry':preview['obstacle_geometry'],'mock_execution':execution}
        print(name+': PASS',flush=True)
        (out_root/'qualification.json').write_text(json.dumps(report,indent=2)+'\n')
    if selected != all_scenarios and not negative_only:
        report['result']='PARTIAL_PASS'
        sys.exit(0)
    # Reachable pose away from the start, then enclose it with a test-only box.
    import yaml
    negative=yaml.safe_load(config.read_text())
    q_block=[-2.5,-2.2,1.9,-1.383,-1.57,0.]
    transform=kinematics.world_transforms(dict(zip(model.JOINT_NAMES,q_block)))['tool0']
    rotation=transform[:3,:3]
    m=np.array([[rotation[0,0]-rotation[1,1]-rotation[2,2],rotation[1,0]+rotation[0,1],rotation[2,0]+rotation[0,2],rotation[2,1]-rotation[1,2]],
        [rotation[1,0]+rotation[0,1],rotation[1,1]-rotation[0,0]-rotation[2,2],rotation[2,1]+rotation[1,2],rotation[0,2]-rotation[2,0]],
        [rotation[2,0]+rotation[0,2],rotation[2,1]+rotation[1,2],rotation[2,2]-rotation[0,0]-rotation[1,1],rotation[1,0]-rotation[0,1]],
        [rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1],np.trace(rotation)]])/3
    eigval,eigvec=np.linalg.eigh(m);orientation=eigvec[:,np.argmax(eigval)].tolist()
    blocked_goal={'position':transform[:3,3].tolist(),'quaternion_xyzw':orientation}
    negative['scenarios']['blocked']={'goal':blocked_goal,'obstacles':{'target_block':{
        'type':'box','position':blocked_goal['position'],'size':[.15,.15,.15]}}}
    negative_file=out_root/'blocked.yaml';negative_file.write_text(yaml.safe_dump(negative))
    out=out_root/'blocked';out.mkdir(exist_ok=True)
    target=load(negative_file,'blocked')['goal'];(out/'target.json').write_text(json.dumps(target))
    environment=dict(os.environ,UR_DOD_ACTION='/move_action',UR_DOD_PIPELINE='isaac_ros_cumotion',
        UR_DOD_LIVE_STATE='1',UR_DOD_URDF=str(root/'ur5e.urdf'))
    environment.update(UR_DOD_OUTPUT_DIR=str(out),UR_DOD_OUTPUT_PREFIX='negative',UR_DOD_COURSE_CONFIG=str(negative_file),
        UR_DOD_COURSE_SCENARIO='blocked',UR_DOD_COURSE_ENABLED='1',UR_DOD_TARGET_TCP=str(out/'target.json'),UR_DOD_SEEDED_IK='0')
    scene.apply(load(negative_file,'blocked'),enabled=False)
    command(['/usr/bin/python3',str(root/'execute-mock-plan.py'),'--prepare','--trajectory',str(out_root/'single_wall/seed.json'),
        '--output-dir',str(out)],out,'reset-mock')
    with (out/'plan.log').open('w') as log:
        failure=subprocess.run(['/usr/bin/python3',str(root/'test-cumotion.py')],env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    negative_report=json.loads((out/'negative-dod.json').read_text())
    if failure.returncode==0 or negative_report.get('planning_status')!='PLANNING FAILED' or (out/'ur5e-negative-planned-trajectory.json').exists():
        raise RuntimeError('Blocked target was not rejected')
    if not negative_report.get('runtime_planner',{}).get('backend_goals'):
        raise RuntimeError('Blocked-target rejection did not reach the GPU backend')
    # Request an infeasible self-colliding joint goal with an empty world to
    # exercise the GPU self-collision model independently of obstacle boxes.
    empty=load(config,'single_wall');scene.apply(empty,enabled=False)
    generator=np.random.default_rng(7);bad=None
    for _ in range(300):
        q=generator.uniform([-1.,-3.,-3.,-3.,-3.,-3.],[1.,0.,3.,3.,3.,3.]).tolist()
        req=GetStateValidity.Request();req.group_name='ur_manipulator'
        req.robot_state.joint_state.name=model.JOINT_NAMES;req.robot_state.joint_state.position=q
        validity=scene.rpc(GetStateValidity,'/check_state_validity',req)
        if not validity.valid and validity.contacts:
            bad=q;contacts=[{'body1':c.contact_body_1,'body2':c.contact_body_2} for c in validity.contacts];break
    if bad is None:raise RuntimeError('No self-collision qualification pose found')
    environment.pop('UR_DOD_TARGET_TCP',None)
    environment.update(UR_DOD_COURSE_CONFIG=str(config),UR_DOD_COURSE_SCENARIO='single_wall',
        UR_DOD_COURSE_ENABLED='0',UR_DOD_JOINT_GOAL=json.dumps(bad),UR_DOD_OUTPUT_PREFIX='self-collision')
    with (out/'self-collision.log').open('w') as log:
        rejected=subprocess.run(['/usr/bin/python3',str(root/'test-cumotion.py')],env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    self_report=json.loads((out/'self-collision-dod.json').read_text())
    if rejected.returncode==0 or self_report.get('moveit_error_code')==1 or not self_report.get('runtime_planner',{}).get('backend_goals'):
        raise RuntimeError('GPU self-collision rejection was not demonstrated: '+str(self_report))
    report['self_collision_test']={'joint_goal':bad,'contacts':contacts,'planner_result':self_report}
    report.update(result='PASS',blocked_target=negative_report)
except Exception as exc:
    report['error']=str(exc)
    raise
finally:
    (out_root/'qualification.json').write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
