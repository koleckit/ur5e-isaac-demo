import json
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState

directory = Path(__file__).resolve().parent
expected = [0.1, -2.2, 1.9, -1.383, -1.57, 0.0]
names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
received = []
rclpy.init()
node = rclpy.create_node('isaac_bridge_dod_client')
publisher = node.create_publisher(JointState, '/ur_demo/bridge_input', 10)
subscription = node.create_subscription(JointState, '/ur_demo/bridge_output', received.append, 10)
report = {'result': 'FAIL'}
try:
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline and len(received) < 3:
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name, message.position = names, expected
        message.velocity, message.effort = [0.0]*6, [0.0]*6
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.02)
    matches = [m for m in received if list(m.name) == names and len(m.position) == 6 and max(abs(a-b) for a,b in zip(m.position, expected)) < 1e-9]
    if len(matches) < 3:
        raise RuntimeError(f'Only {len(matches)} matching replies from Sim bridge')
    report.update(result='PASS', matching_replies=len(matches), positions=list(matches[-1].position))
    print(json.dumps(report, indent=2))
except Exception as exc:
    report['error'] = str(exc)
    raise
finally:
    (directory / 'isaac-bridge-client-dod.json').write_text(json.dumps(report, indent=2) + '\n')
    node.destroy_node()
    rclpy.shutdown()
