#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$project_dir"
exec > >(tee ur-mock-dod.log) 2>&1
date -Is
source "$project_dir/ros-env.sh"
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_moveit_config
python3 "$project_dir/test-ur-mock.py"
