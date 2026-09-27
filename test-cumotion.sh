#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$project_dir"
if sg docker -c 'docker container inspect ur_demo_cumotion_dod' >/dev/null 2>&1; then
  echo 'An existing cuMotion DoD container is present; refusing to replace it.' >&2
  exit 1
fi
bash "$project_dir/launch-cumotion-dod.sh" > "$project_dir/cumotion-planner.log" 2>&1 &
planner_pid=$!
cleanup() {
  sg docker -c 'docker stop --time 10 ur_demo_cumotion_dod' >/dev/null 2>&1 || true
  wait "$planner_pid" 2>/dev/null || true
}
trap cleanup EXIT
source "$project_dir/ros-env.sh"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=173
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
if [[ ${UR_CUMOTION_CLIENT:-host} == container ]]; then
  for _ in $(seq 1 50); do
    if sg docker -c 'docker inspect --format "{{.State.Running}}" ur_demo_cumotion_dod' 2>/dev/null | grep -qx true; then break; fi
    sleep 0.2
  done
  sg docker -c "docker exec ur_demo_cumotion_dod bash -lc 'source /opt/ros/lyrical/setup.bash; python3 /demo/test-cumotion.py'" 2>&1 | tee "$project_dir/cumotion-dod.log"
else
  python3 "$project_dir/test-cumotion.py" 2>&1 | tee "$project_dir/cumotion-dod.log"
fi
