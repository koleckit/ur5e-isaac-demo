#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export ROS_DOMAIN_ID=172 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
setsid bash "$project_dir/launch-ur-mock.sh" > "$project_dir/twins-mock.log" 2>&1 &
mock_pid=$!
cleanup() {
  kill -INT -- "-$mock_pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$mock_pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$mock_pid" 2>/dev/null || true
  wait "$mock_pid" 2>/dev/null || true
}
trap cleanup EXIT
printf '{"result":"RUNNING"}\n' > "$project_dir/twins-dod.json"
timeout --kill-after=5 120 bash "$project_dir/run-isaac-python.sh" "$project_dir/isaac-twins.py" --headless --test > "$project_dir/twins-dod.log" 2>&1
cat "$project_dir/twins-dod.json"
python3 "$project_dir/check-dod.py" "$project_dir/twins-dod.json"
