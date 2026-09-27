#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$project_dir"
if sg docker -c 'docker inspect ur_demo_moveit_cumotion_dod' >/dev/null 2>&1; then
  echo 'An existing MoveIt DoD container is present; refusing to replace it.' >&2; exit 1
fi
container_uid=$(id -u)
container_gid=$(id -g)
sg docker -c "docker run --rm --user $container_uid:$container_gid -e HOME=/tmp/ur-demo --gpus all --network host --ipc host --name ur_demo_moveit_cumotion_dod -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=173 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v '$project_dir:/demo' --entrypoint bash ur-demo:isaac-ros-5.0 -lc 'mkdir -p /tmp/ur-demo; source /opt/ros/lyrical/setup.bash; ros2 launch /demo/cumotion-moveit.launch.py'" > moveit-cumotion-planner.log 2>&1 &
planner_pid=$!
cleanup() {
  sg docker -c 'docker stop --time 10 ur_demo_moveit_cumotion_dod' >/dev/null 2>&1 || true
  wait "$planner_pid" 2>/dev/null || true
}
trap cleanup EXIT
source "$project_dir/ros-env.sh"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=173
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
export UR_DOD_ACTION=/move_action
export UR_DOD_PIPELINE=isaac_ros_cumotion
export UR_DOD_OUTPUT_PREFIX=${UR_DOD_OUTPUT_PREFIX:-moveit-cumotion}
python3 "$project_dir/test-cumotion.py" 2>&1 | tee "${UR_DOD_OUTPUT_PREFIX}-dod.log"
