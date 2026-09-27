#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
domain=${ROS_DOMAIN_ID:-178}
[[ $domain == 178 || $domain == 174 ]]
name=ur_demo_course_planner_$domain
if sg docker -c "docker inspect $name" >/dev/null 2>&1; then
  echo 'Course planner container already exists; close its owning launcher first.' >&2
  exit 1
fi
urdf=/demo/ur5e.urdf
if [[ $domain == 174 ]]; then urdf=/demo/robot-setup/ur5e-calibrated.urdf; fi
exec sg docker -c "docker run --rm --user $(id -u):$(id -g) --gpus all --network host --ipc host --name $name -e HOME=/tmp/course -e ROS_DOMAIN_ID=$domain -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e RMW_FASTRTPS_USE_QOS_FROM_XML=1 -e FASTRTPS_DEFAULT_PROFILES_FILE=/demo/physical-fastdds.xml -e UR_DEMO_URDF=$urdf -e UR_DEMO_XRDF=/demo/obstacle_course/runs/model/ur5e-course.xrdf -e UR_DEMO_SELF_SPHERES=true -v '$root:/demo' --entrypoint bash ur-demo:isaac-ros-5.0 -lc 'mkdir -p /tmp/course; source /opt/ros/lyrical/setup.bash; python3 /demo/obstacle_course/scripts/prepare.py --output-dir /demo/obstacle_course/runs/model --urdf $urdf --vendor-xrdf /opt/ros/lyrical/share/isaac_ros_cumotion_robot_description/xrdf/ur5e.xrdf; ros2 launch /demo/cumotion-moveit.launch.py'"
