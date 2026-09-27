"""Refuse to launch a second planning stack or take over another mock driver."""
import os
import time
import rclpy
rclpy.init();node=rclpy.create_node('course_domain_preflight')
try:
    until=time.monotonic()+3
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
    forbidden={'move_group','cumotion_planner'}
    if os.environ.get('ROS_DOMAIN_ID')=='178':forbidden.add('controller_manager')
    found=sorted({name for name,namespace in node.get_node_names_and_namespaces()} & forbidden)
    if found:raise RuntimeError('Close existing processes in this course domain first: '+str(found))
finally:
    node.destroy_node();rclpy.shutdown()
