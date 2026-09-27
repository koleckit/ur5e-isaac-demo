"""Choose a nearby collision-free TCP goal configuration; never add path waypoints."""
import copy
import math
import xml.etree.ElementTree as ET
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

def solve(client, request, wait, names, start, urdf):
    limits={j.get('name'):j.find('limit') for j in ET.parse(urdf).getroot().findall('joint')}
    candidates=[]
    # Numerical IK seeds only. These are not trajectories or intermediate goals.
    for pan_offset in (0., math.pi/2, -math.pi/2, math.pi):
        query=copy.deepcopy(request)
        seed=list(start)
        seed[0]=max(float(limits[names[0]].get('lower')),
            min(float(limits[names[0]].get('upper')), seed[0]+pan_offset))
        query.ik_request.robot_state.joint_state.position=seed
        pending=client.call_async(query);wait(pending.done,10,'Course IK timeout')
        result=pending.result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:continue
        raw=dict(zip(result.solution.joint_state.name,result.solution.joint_state.position))
        normalized=[]
        for name,current in zip(names,start):
            lower,upper=float(limits[name].get('lower')),float(limits[name].get('upper'))
            equivalent=[raw[name]+2*math.pi*k for k in range(-2,3) if lower<=raw[name]+2*math.pi*k<=upper]
            if not equivalent:break
            normalized.append(min(equivalent,key=lambda q:abs(q-current)))
        if len(normalized)==len(names):candidates.append(normalized)
    if not candidates:raise RuntimeError('No collision-free TCP IK solution')
    distinct=[]
    for candidate in sorted(candidates,key=lambda q:sum((a-b)**2 for a,b in zip(q,start))):
        if not any(max(abs(a-b) for a,b in zip(candidate,old))<.01 for old in distinct):
            distinct.append(candidate)
    return [dict(zip(names,c)) for c in distinct], {
        'candidate_count':len(distinct),'selection':'automatic GPU planning over bounded IK branches, nearest first'}
