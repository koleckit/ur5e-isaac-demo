#!/usr/bin/env bash
# Source explicitly in shells used for this project.
if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
  echo 'ROS Lyrical is not installed yet.' >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/lyrical/setup.bash
