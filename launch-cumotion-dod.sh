#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# The container is removed after the planner exits; all packages live in its image.
container_uid=$(id -u)
container_gid=$(id -g)
exec sg docker -c "docker run --rm --user $container_uid:$container_gid -e HOME=/tmp/ur-demo --gpus all --network host --ipc host --name ur_demo_cumotion_dod -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e ROS_DOMAIN_ID=173 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v '$project_dir:/demo' --entrypoint bash ur-demo:isaac-ros-5.0 -lc 'set -e; mkdir -p /tmp/ur-demo; source /opt/ros/lyrical/setup.bash; export PATH=/usr/local/cuda/bin:\$PATH; xacro -o /demo/ur5e.urdf /opt/ros/lyrical/share/ur_description/urdf/ur.urdf.xacro ur_type:=ur5e name:=ur5e; ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py cumotion_action_server.urdf_file_path:=/demo/ur5e.urdf cumotion_action_server.xrdf_file_path:=/opt/ros/lyrical/share/isaac_ros_cumotion_robot_description/xrdf/ur5e.xrdf cumotion_action_server.read_esdf_world:=false'"
