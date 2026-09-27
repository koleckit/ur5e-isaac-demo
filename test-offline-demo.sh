#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/ros-env.sh"
trajectory=${UR_DEMO_TRAJECTORY:-ur5e-moveit-cumotion-planned-trajectory.json}
export ROS_DOMAIN_ID=172 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4 UR_MOCK_CUMOTION=1
unset ROS_STATIC_PEERS
# Permit cuMotion's terminal velocity only in the dedicated mock configuration.
python3 - "$project_dir" <<'PY'
import sys
from pathlib import Path
p=Path(sys.argv[1])
config=Path('/opt/ros/lyrical/share/ur_robot_driver/config/ur_controllers.yaml').read_text()
needle='\njoint_trajectory_controller:\n  ros__parameters:\n'
assert config.count(needle)==1
# Keep vendor quoting: tf_prefix expands to an empty string in this mock.
config=config.replace(needle, needle+'    allow_nonzero_velocity_at_trajectory_end: true\n', 1)
(p/'mock-cumotion-controllers.yaml').write_text(config)
PY
setsid bash "$project_dir/launch-ur-mock.sh" > "$project_dir/offline-mock.log" 2>&1 &
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
python3 "$project_dir/execute-mock-plan.py" --prepare --trajectory "$trajectory" > "$project_dir/mock-prepare-dod.log" 2>&1
printf '{"result":"RUNNING"}\n' > "$project_dir/twins-dod.json"
timeout --kill-after=5 120 bash "$project_dir/run-isaac-python.sh" "$project_dir/isaac-twins.py" --headless --test --trajectory "$trajectory" > "$project_dir/twins-dod.log" 2>&1
python3 "$project_dir/check-dod.py" "$project_dir/twins-dod.json"
python3 "$project_dir/execute-mock-plan.py" --trajectory "$trajectory" > "$project_dir/mock-execute-dod.log" 2>&1
python3 "$project_dir/check-dod.py" "$project_dir/mock-prepare-dod.json" "$project_dir/mock-execute-dod.json"
python3 "$project_dir/verify-tracking.py"
cat "$project_dir/mock-execute-dod.json"
