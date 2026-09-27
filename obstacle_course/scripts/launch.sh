#!/usr/bin/env bash
# Course owns its planner and mock; physical driver is never started or moved here.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
scenario=${1:-single_wall}
mode=${2:-virtual}
[[ $scenario == single_wall || $scenario == corridor || $scenario == gate_3d ]]
[[ $mode == virtual || $mode == physical || $mode == qualify ]]
if [[ $mode == physical && ${UR_COURSE_WORKFLOW_TEST:-0} == 1 ]]; then
  echo 'Automated workflow tests are mock-only.' >&2
  exit 1
fi
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH OLD_PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD CMAKE_PREFIX_PATH
source "$root/ros-env.sh"
export ROS_DOMAIN_ID=178 UR_MOCK_ROS_DOMAIN_ID=178 UR_DEMO_ROS_DOMAIN_ID=178
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4 UR_MOCK_CUMOTION=1
unset ROS_STATIC_PEERS
out="$root/obstacle_course/runs/$scenario"
mkdir -p "$out"
mock_pid=''
planner_pid=''
cleanup() {
  if [[ -n $planner_pid ]]; then
    sg docker -c "docker stop --time 10 ur_demo_course_planner_$ROS_DOMAIN_ID" >/dev/null 2>&1 || true
    wait "$planner_pid" 2>/dev/null || true
  fi
  if [[ -n $mock_pid ]]; then
    kill -INT -- "-$mock_pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$mock_pid" 2>/dev/null || break
      sleep .1
    done
    kill -TERM -- "-$mock_pid" 2>/dev/null || true
    wait "$mock_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
python3 "$root/obstacle_course/scripts/prepare.py" --scenario "$scenario" --output-dir "$out"
sim_args=(--obstacle-course "$root/obstacle_course/config/obstacle_course.yaml" --scenario "$scenario" --output-dir "$out")
if [[ $mode == physical ]]; then
  export ROS_DOMAIN_ID=174 UR_DEMO_ROS_DOMAIN_ID=174
  python3 "$root/obstacle_course/scripts/check-domain.py"
  # Require the existing calibrated driver and inactive physical controllers.
  python3 "$root/robot-setup/test-calibrated-fk.py" > "$out/physical-fk.log" 2>&1
  python3 "$root/prepare-physical-preview.py" > "$out/physical-preflight.log" 2>&1
  sim_args+=(--physical-preview --enable-physical-execution --trajectory "$root/robot-setup/physical-seed-trajectory.json")
else
  python3 "$root/obstacle_course/scripts/check-domain.py"
  setsid bash "$root/launch-ur-mock.sh" > "$out/mock.log" 2>&1 & mock_pid=$!
  python3 "$root/execute-mock-plan.py" --prepare --trajectory "$out/seed.json" --output-dir "$out" > "$out/prepare.log" 2>&1
  sim_args+=(--trajectory "$out/seed.json")
fi
bash "$root/obstacle_course/launch/planner.sh" > "$out/planner.log" 2>&1 & planner_pid=$!
if [[ $mode == qualify ]]; then
  python3 "$root/obstacle_course/scripts/qualify.py" "$out"
else
  if [[ ${UR_COURSE_WORKFLOW_TEST:-0} == 1 && $mode == virtual ]]; then
    sim_args+=(--headless --workflow-test)
  elif [[ ${UR_COURSE_TEST:-0} == 1 ]]; then
    sim_args+=(--headless --course-test)
  fi
  bash "$root/run-isaac-python.sh" "$root/isaac-twins.py" "${sim_args[@]}" > "$out/isaac.log" 2>&1
fi
