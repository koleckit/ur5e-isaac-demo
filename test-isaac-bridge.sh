#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/ros-env.sh"
export ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS
python3 "$project_dir/test-isaac-bridge-client.py" > "$project_dir/isaac-bridge-client-dod.log" 2>&1 &
client_pid=$!
trap 'kill "$client_pid" 2>/dev/null || true; wait "$client_pid" 2>/dev/null || true' EXIT
sim_dir=${ISAAC_SIM_DIR:-$HOME/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64}
printf '{"result":"RUNNING"}\n' > "$project_dir/isaac-bridge-dod.json"
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u CMAKE_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
  ROS_DISTRO=jazzy LD_LIBRARY_PATH="$sim_dir/exts/isaacsim.ros2.core/jazzy/lib" \
  timeout --kill-after=5 120 "$sim_dir/python.sh" -u "$project_dir/test-isaac-bridge.py" > "$project_dir/isaac-bridge-dod.log" 2>&1
wait "$client_pid"
cat "$project_dir/isaac-bridge-dod.json" "$project_dir/isaac-bridge-client-dod.json"
python3 "$project_dir/check-dod.py" "$project_dir/isaac-bridge-dod.json" "$project_dir/isaac-bridge-client-dod.json"
