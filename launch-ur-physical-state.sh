#!/usr/bin/env bash
# Physical UR5e connection and state acquisition; motion controllers stay inactive.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH OLD_PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CMAKE_PREFIX_PATH
source "$project_dir/ros-env.sh"
# Separate physical hardware from the offline mock (domain 172).
export ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS
extra_args=()
if [[ -f "$project_dir/robot-setup/ur5e-calibration.yaml" ]]; then
  extra_args+=("kinematics_params_file:=$project_dir/robot-setup/ur5e-calibration.yaml")
fi
exec ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=192.168.60.201 reverse_ip:=192.168.60.200 \
  use_mock_hardware:=false headless_mode:=false \
  activate_joint_controller:=false initial_joint_controller:=joint_trajectory_controller \
  launch_rviz:=false launch_dashboard_client:=false use_tool_communication:=false \
  script_sender_port:=50002 controller_spawner_timeout:=60 "${extra_args[@]}"
