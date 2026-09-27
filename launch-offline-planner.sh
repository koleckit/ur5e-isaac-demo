#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if sg docker -c 'docker inspect ur_demo_offline_planner' >/dev/null 2>&1; then
  echo 'Offline planner container already exists; refusing to replace it.' >&2
  exit 1
fi
container_uid=$(id -u)
container_gid=$(id -g)
exec sg docker -c "docker run --rm --user $container_uid:$container_gid -e HOME=/tmp/ur-demo --gpus all --network host --ipc host --name ur_demo_offline_planner -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=172 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v '$project_dir:/demo' --entrypoint bash ur-demo:isaac-ros-5.0 -lc 'mkdir -p /tmp/ur-demo; source /opt/ros/lyrical/setup.bash; ros2 launch /demo/cumotion-moveit.launch.py'"
