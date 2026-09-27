#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/ros-env.sh"
export ROS_DOMAIN_ID=172 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4 UR_MOCK_CUMOTION=1
unset ROS_STATIC_PEERS
[[ -f "$project_dir/mock-cumotion-controllers.yaml" ]]
setsid bash "$project_dir/launch-ur-mock.sh" > "$project_dir/interactive-mock.log" 2>&1 &
mock_pid=$!
planner_pid=''
cleanup() {
  if [[ -n "$planner_pid" ]]; then
    sg docker -c 'docker stop --time 10 ur_demo_offline_planner' >/dev/null 2>&1 || true
    wait "$planner_pid" 2>/dev/null || true
  fi
  kill -INT -- "-$mock_pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$mock_pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$mock_pid" 2>/dev/null || true
  wait "$mock_pid" 2>/dev/null || true
}
trap cleanup EXIT
python3 "$project_dir/execute-mock-plan.py" --prepare > "$project_dir/interactive-prepare.log" 2>&1
bash "$project_dir/launch-offline-planner.sh" > "$project_dir/interactive-planner.log" 2>&1 &
planner_pid=$!
printf '{"result":"RUNNING"}\n' > "$project_dir/twins-dod.json"
if [[ ${UR_DEMO_WORKFLOW_TEST:-0} == 1 ]]; then
  sim_test_args=(--workflow-test)
  if [[ ${UR_DEMO_TEST_GUI:-0} != 1 ]]; then sim_test_args+=(--headless); fi
  timeout --kill-after=5 180 bash "$project_dir/run-isaac-python.sh" "$project_dir/isaac-twins.py" "${sim_test_args[@]}" > "$project_dir/interactive-sim.log" 2>&1
  python3 "$project_dir/check-dod.py" "$project_dir/interactive-cumotion-dod.json" "$project_dir/twins-dod.json" "$project_dir/mock-execute-dod.json"
  python3 "$project_dir/verify-tracking.py"
  cat "$project_dir/twins-dod.json"
else
  bash "$project_dir/run-isaac-python.sh" "$project_dir/isaac-twins.py" > "$project_dir/interactive-sim.log" 2>&1
fi
