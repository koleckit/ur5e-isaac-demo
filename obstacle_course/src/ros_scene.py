"""Apply/read back owned BOX objects; verify geometry and the real cuMotion pipeline."""
import time
import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, QueryPlannerInterfaces
from shape_msgs.msg import SolidPrimitive
from rcl_interfaces.srv import GetParameters
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy
from world import matrix

def pose(position, orientation):
    result = Pose()
    result.position.x, result.position.y, result.position.z = position
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = orientation
    return result

def object_for(box):
    obj = CollisionObject(id=box['id'], operation=CollisionObject.ADD)
    obj.header.frame_id = 'base_link'
    obj.pose.orientation.w = 1.0
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=box['size'])]
    obj.primitive_poses = [pose(box['position'],box['quaternion_xyzw'])]
    return obj

class Scene:
    def __init__(self, node, wait):
        self.node, self.wait = node, wait

    def rpc(self, kind, name, request=None):
        client = self.node.create_client(kind,name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise TimeoutError('Missing service '+name)
            future = client.call_async(request or kind.Request())
            self.wait(future.done,10,'Service timeout '+name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def snapshot(self):
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.TRANSFORMS |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS | PlanningSceneComponents.LINK_PADDING_AND_SCALING)
        return self.rpc(GetPlanningScene,'/get_planning_scene',request).scene

    def apply(self, course, enabled=True):
        old = self.snapshot()
        if any(not o.id.startswith('oc_') for o in old.world.collision_objects):
            raise RuntimeError('Course needs its own scene; foreign world objects would be invisible in Isaac Sim')
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff = True
        diff.world.collision_objects = [CollisionObject(id=o.id,operation=CollisionObject.REMOVE)
                                        for o in old.world.collision_objects if o.id.startswith('oc_')]
        if enabled:
            diff.world.collision_objects.extend(object_for(box) for box in course['obstacles'])
        request = ApplyPlanningScene.Request(scene=diff)
        if not self.rpc(ApplyPlanningScene,'/apply_planning_scene',request).success:
            raise RuntimeError('MoveIt rejected the obstacle scene')
        actual = self.snapshot()
        self.verify(course,actual,enabled)
        return actual

    def verify(self, course, actual=None, enabled=True):
        actual = actual or self.snapshot()
        expected = {box['id']:box for box in course['obstacles']} if enabled else {}
        observed = {obj.id:obj for obj in actual.world.collision_objects}
        if set(expected) != set(observed):
            raise RuntimeError('Planning Scene object IDs differ from the shared course configuration')
        for name,box in expected.items():
            obj = observed[name]
            if obj.header.frame_id not in ('base_link','world') or len(obj.primitives)!=1 or obj.meshes or obj.planes:
                raise RuntimeError('Unexpected obstacle frame/geometry: '+name)
            if obj.primitives[0].type != SolidPrimitive.BOX or not np.allclose(obj.primitives[0].dimensions,box['size'],atol=1e-9):
                raise RuntimeError('Obstacle dimensions differ: '+name)
            def transform(p):
                return matrix([p.position.x,p.position.y,p.position.z],
                              [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            observed_pose = transform(obj.pose) @ transform(obj.primitive_poses[0])
            if not np.allclose(observed_pose,matrix(box['position'],box['quaternion_xyzw']),atol=1e-8):
                raise RuntimeError('Obstacle pose differs: '+name)
        # Collision objects must not be bypassed via the AllowedCollisionMatrix.
        acm = actual.allowed_collision_matrix
        for i,name in enumerate(acm.entry_names):
            if name in expected and any(acm.entry_values[i].enabled):
                raise RuntimeError('Obstacle collision disabled in ACM: '+name)
        if any(name in expected and allowed for name,allowed in zip(acm.default_entry_names,acm.default_entry_values)):
            raise RuntimeError('Obstacle collisions disabled by an ACM default')
        return actual

class RuntimeProof:
    def __init__(self,node,scene):
        self.node, self.scene, self.events, self.previous = node, scene, {}, set()
        self.goal_times = {}
        self.subscription = node.create_subscription(GoalStatusArray,'/cumotion/move_group/_action/status',self.status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def status(self,message):
        for status in message.status_list:
            key = bytes(status.goal_info.goal_id.uuid).hex()
            self.events[key] = status.status
            self.goal_times[key] = status.goal_info.stamp.sec*10**9+status.goal_info.stamp.nanosec

    def begin(self):
        values = self.scene.rpc(GetParameters,'/move_group/get_parameters',
            GetParameters.Request(names=['default_planning_pipeline','isaac_ros_cumotion.planning_plugins'])).values
        if values[0].string_value != 'isaac_ros_cumotion' or 'isaac_ros_cumotion_moveit/CumotionPlanner' not in values[1].string_array_value:
            raise RuntimeError('cuMotion is not the active MoveIt pipeline/plugin')
        self.scene.wait(lambda: any(p.node_name=='cumotion_planner' for p in
            self.node.get_publishers_info_by_topic('/cumotion/move_group/_action/status')),
            10,'cuMotion backend status discovery timeout')
        backend = self.node.get_publishers_info_by_topic('/cumotion/move_group/_action/status')
        if len(backend)!=1 or backend[0].node_name!='cumotion_planner':
            raise RuntimeError('Expected exactly one real cuMotion backend: '+str([p.node_name for p in backend]))
        params = self.scene.rpc(GetParameters,'/cumotion_planner/get_parameters',
            GetParameters.Request(names=['read_esdf_world','xrdf_file_path'])).values
        if params[0].bool_value:
            raise RuntimeError('Perception ESDF must be disabled in the static course')
        interfaces = self.scene.rpc(QueryPlannerInterfaces,'/query_planner_interface').planner_interfaces
        self.proof = {'pipeline':values[0].string_value,'plugin':values[1].string_array_value,
            'backend_node':backend[0].node_name,'read_esdf_world':False,'xrdf_file':params[1].string_value,
            'interfaces':[{'name':p.name,'planner_ids':list(p.planner_ids)} for p in interfaces]}
        for _ in range(10):
            rclpy.spin_once(self.node,timeout_sec=.05)
        self.previous = set(self.events)

    def arm(self):
        # Ignore late transient replays of a preceding request by server timestamp.
        self.request_ns = self.node.get_clock().now().nanoseconds

    def new_goals(self):
        return [{'uuid':key,'status':value,'accepted_ros_time_ns':self.goal_times[key]}
                for key,value in self.events.items() if key not in self.previous and self.goal_times[key]>=self.request_ns]

    def finish(self, wait):
        wait(lambda: any(g['status'] in (4,5,6) for g in self.new_goals()),
             5,'No terminal cuMotion backend goal observed for this MoveIt request')
        self.proof['backend_goals'] = self.new_goals()
        if len(self.proof['backend_goals']) != 1:
            raise RuntimeError('Ambiguous cuMotion request trace; expected one new backend goal')
        return self.proof
