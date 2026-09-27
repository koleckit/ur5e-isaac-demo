#!/usr/bin/env bash
set -eo pipefail
sim_dir=${ISAAC_SIM_DIR:-$HOME/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64}
# Keep Isaac Sim's supported bundled Jazzy libraries separate from host Lyrical.
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u CMAKE_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
  ROS_DISTRO=jazzy LD_LIBRARY_PATH="$sim_dir/exts/isaacsim.ros2.core/jazzy/lib" \
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  "$sim_dir/python.sh" -u "$@"
