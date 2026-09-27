#!/usr/bin/env bash
# Calibrated physical display; Execute is an explicit opt-in in launch-ur-demo.sh.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH OLD_PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CMAKE_PREFIX_PATH
source "$project_dir/ros-env.sh"
export ROS_DOMAIN_ID=174 UR_DEMO_ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
unset ROS_STATIC_PEERS
python3 "$project_dir/robot-setup/test-calibrated-fk.py" > "$project_dir/robot-setup/physical-preview-fk.log" 2>&1
python3 "$project_dir/prepare-physical-preview.py" > "$project_dir/robot-setup/physical-preview-preflight.log" 2>&1
bash "$project_dir/launch-physical-planner.sh" > "$project_dir/robot-setup/physical-planner.log" 2>&1 &
planner_pid=$!
cleanup() {
  # Stop only the container started by this launcher; the physical driver stays running.
  if kill -0 "$planner_pid" 2>/dev/null; then
    sg docker -c 'docker stop --time 10 ur_demo_physical_planner' >/dev/null 2>&1 || true
  fi
  wait "$planner_pid" 2>/dev/null || true
}
trap cleanup EXIT
python3 - <<'PY'
import time
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
rclpy.init()
node = rclpy.create_node('ur_physical_planner_startup_check')
clients = [ActionClient(node, MoveGroup, name) for name in ('/move_action', '/cumotion/move_group')]
deadline = time.monotonic()+60
while not all(client.server_is_ready() for client in clients):
    if time.monotonic() > deadline:
        raise TimeoutError('Physical planner backend did not become ready; check physical-planner.log')
    rclpy.spin_once(node, timeout_sec=.1)
print('MoveIt and cuMotion action servers ready')
node.destroy_node()
rclpy.shutdown()
PY
sim_args=(--physical-preview)
if [[ ${UR_DEMO_PHYSICAL_EXECUTE:-0} == 1 ]]; then sim_args+=(--enable-physical-execution); fi
if [[ ${UR_DEMO_PHYSICAL_PREVIEW_TEST:-0} == 1 ]]; then
  sim_args+=(--plan-preview-test)
  if [[ ${UR_DEMO_TEST_GUI:-0} != 1 ]]; then sim_args+=(--headless); fi
  timeout --kill-after=5 180 bash "$project_dir/run-isaac-python.sh" "$project_dir/isaac-twins.py" "${sim_args[@]}" > "$project_dir/robot-setup/physical-sim.log" 2>&1
  python3 "$project_dir/check-dod.py" "$project_dir/physical-twins-dod.json" "$project_dir/physical-interactive-cumotion-dod.json" "$project_dir/physical-preview-attestation.json"
else
  bash "$project_dir/run-isaac-python.sh" "$project_dir/isaac-twins.py" "${sim_args[@]}" > "$project_dir/robot-setup/physical-sim.log" 2>&1
fi
