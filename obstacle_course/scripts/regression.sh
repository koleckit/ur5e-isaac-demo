#!/usr/bin/env bash
# Preserve the original offline workflow and its guarded executor qualification.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
UR_DEMO_WORKFLOW_TEST=1 bash "$root/launch-offline-demo.sh"
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH OLD_PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CMAKE_PREFIX_PATH
source "$root/ros-env.sh"
export ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS
python3 - <<'PY'
import time,rclpy
rclpy.init();node=rclpy.create_node('executor_mock_domain_guard')
until=time.monotonic()+3
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
if 'controller_manager' in node.get_node_names():raise RuntimeError('Domain 176 already has a controller manager')
node.destroy_node();rclpy.shutdown()
PY
setsid bash -c 'source "$1/ros-env.sh"; exec ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 use_mock_hardware:=true launch_rviz:=false launch_dashboard_client:=false activate_joint_controller:=false initial_joint_controller:=joint_trajectory_controller' _ "$root" > "$root/obstacle_course/runs/regression-mock.log" 2>&1 &
mock_pid=$!
cleanup() {
  kill -INT -- "-$mock_pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$mock_pid" 2>/dev/null || break
    sleep .1
  done
  kill -TERM -- "-$mock_pid" 2>/dev/null || true
  wait "$mock_pid" 2>/dev/null || true
}
trap cleanup EXIT
python3 "$root/test-physical-executor.py" > "$root/obstacle_course/runs/regression-executor.log" 2>&1
python3 "$root/check-dod.py" "$root/twins-dod.json" "$root/physical-executor-mock-dod.json"
