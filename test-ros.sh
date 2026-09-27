#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$project_dir"
exec > >(tee ros-dod.log) 2>&1
date -Is
source "$project_dir/ros-env.sh"
[[ ${ROS_DISTRO:-} == lyrical ]]
ros2 pkg prefix rclpy
ros2 pkg prefix demo_nodes_cpp
python3 "$project_dir/test-ros-pubsub.py"
