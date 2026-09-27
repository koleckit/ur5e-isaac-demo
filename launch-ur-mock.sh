#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/ros-env.sh"
export ROS_DOMAIN_ID=${UR_MOCK_ROS_DOMAIN_ID:-172}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
extra_args=()
if [[ ${UR_MOCK_CUMOTION:-0} == 1 ]]; then
  extra_args+=("controllers_file:=$project_dir/mock-cumotion-controllers.yaml")
fi
exec ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 use_mock_hardware:=true launch_rviz:=false launch_dashboard_client:=false initial_joint_controller:=joint_trajectory_controller controller_spawner_timeout:=30 "${extra_args[@]}"
