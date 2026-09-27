#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# This wrapper may be called from Isaac Sim's bundled Python/Jazzy environment.
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH OLD_PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CMAKE_PREFIX_PATH
source "$project_dir/ros-env.sh"
export ROS_DOMAIN_ID=${UR_DEMO_ROS_DOMAIN_ID:-172} FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
exec /usr/bin/python3 "$@"
