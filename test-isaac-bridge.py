"""Verify actual ROS OmniGraph input/output against a separate Lyrical process."""
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
import json
import os
import time
from pathlib import Path
import omni.graph.core as og
import omni.kit.app
import omni.timeline

directory = Path(__file__).resolve().parent
report = {'result': 'FAIL', 'sim_ros_distro': os.environ.get('ROS_DISTRO'), 'peer_ros_distro': 'lyrical'}
try:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate('isaacsim.ros2.nodes', True)
    app.update()
    og.Controller.edit({'graph_path': '/BridgeDoD', 'evaluator_name': 'execution'}, {
        og.Controller.Keys.CREATE_NODES: [
            ('Tick', 'omni.graph.action.OnPlaybackTick'),
            ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
            ('Input', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
            ('Output', 'isaacsim.ros2.bridge.ROS2PublishJointState')],
        og.Controller.Keys.CONNECT: [
            ('Tick.outputs:tick', 'Input.inputs:execIn'),
            ('Input.outputs:execOut', 'Output.inputs:execIn'),
            ('Context.outputs:context', 'Input.inputs:context'),
            ('Context.outputs:context', 'Output.inputs:context'),
            ('Input.outputs:jointNames', 'Output.inputs:jointNames'),
            ('Input.outputs:positionCommand', 'Output.inputs:jointPositions'),
            ('Input.outputs:velocityCommand', 'Output.inputs:jointVelocities'),
            ('Input.outputs:effortCommand', 'Output.inputs:jointEfforts'),
            ('Input.outputs:timeStamp', 'Output.inputs:timeStamp')],
        og.Controller.Keys.SET_VALUES: [
            ('Context.inputs:useDomainIDEnvVar', True),
            ('Input.inputs:topicName', '/ur_demo/bridge_input'),
            ('Output.inputs:topicName', '/ur_demo/bridge_output'),
            ('Output.inputs:jointDofTypes', [0]*6),
            ('Output.inputs:stageMetersPerUnit', 1.0)]})
    omni.timeline.get_timeline_interface().play()
    deadline = time.monotonic() + 40
    received = 0
    while time.monotonic() < deadline and received < 10:
        app.update()
        names = og.Controller.get(og.Controller.attribute('/BridgeDoD/Input.outputs:jointNames'))
        values = og.Controller.get(og.Controller.attribute('/BridgeDoD/Input.outputs:positionCommand'))
        if len(names) == 6 and len(values) == 6:
            received += 1
    if received < 10:
        raise TimeoutError('OmniGraph did not receive six-joint ROS messages')
    report.update(result='PASS', received_frames=received, joint_names=list(names), positions=list(values))
    # Allow the independent client to observe the bridge publisher.
    for _ in range(100):
        app.update()
    print('ISAAC_BRIDGE_DOD:', json.dumps(report))
except Exception as exc:
    report.update(result='FAIL', error=str(exc))
    raise
finally:
    (directory / 'isaac-bridge-dod.json').write_text(json.dumps(report, indent=2) + '\n')
    app.close()
