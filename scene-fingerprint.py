"""Fingerprint collision-relevant MoveIt scene data, excluding live joint positions."""
import hashlib
import json
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState, MultiDOFJointState
from builtin_interfaces.msg import Time
from moveit_msgs.msg import PlanningSceneComponents

COMPONENTS = (PlanningSceneComponents.SCENE_SETTINGS |
              PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
              PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
              PlanningSceneComponents.OCTOMAP |
              PlanningSceneComponents.TRANSFORMS |
              PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
              PlanningSceneComponents.LINK_PADDING_AND_SCALING)

def fingerprint(scene):
    scene.robot_state.joint_state = JointState()
    scene.robot_state.multi_dof_joint_state = MultiDOFJointState()
    for obj in scene.world.collision_objects:
        obj.header.stamp = Time()
    for attached in scene.robot_state.attached_collision_objects:
        attached.object.header.stamp = Time()
    # MoveIt includes unrelated live TFs (e.g. tool0_controller) here.
    # Only transforms of world collision geometry/octomap affect this collision scene.
    frames = {obj.header.frame_id for obj in scene.world.collision_objects}
    if scene.world.octomap.octomap.data:
        frames.add(scene.world.octomap.header.frame_id)
    scene.fixed_frame_transforms = [t for t in scene.fixed_frame_transforms if t.header.frame_id in frames]
    for transform in scene.fixed_frame_transforms:
        transform.header.stamp = Time()
    scene.world.octomap.header.stamp = Time()
    data = message_to_ordereddict(scene)
    # CDR contains alignment padding; hash canonical content, not binary padding.
    data.pop('name', None)
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
