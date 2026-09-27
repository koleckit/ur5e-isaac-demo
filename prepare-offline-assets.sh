#!/usr/bin/env bash
# Generate a generic UR5e asset and an explicitly synthetic mock seed.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/ros-env.sh"
xacro /opt/ros/lyrical/share/ur_description/urdf/ur.urdf.xacro \
  ur_type:=ur5e name:=ur5e > "$project_dir/ur5e.urdf"
python3 - "$project_dir" <<'PY'
from pathlib import Path
import sys
project = Path(sys.argv[1])
config = Path('/opt/ros/lyrical/share/ur_robot_driver/config/ur_controllers.yaml').read_text()
needle = '\njoint_trajectory_controller:\n  ros__parameters:\n'
assert config.count(needle) == 1
config = config.replace(needle, needle+'    allow_nonzero_velocity_at_trajectory_end: true\n', 1)
(project / 'mock-cumotion-controllers.yaml').write_text(config)
# Only the mock launcher uses this synthetic initialization trajectory.
(project / 'ur5e-moveit-cumotion-planned-trajectory.json').write_bytes(
    (project / 'tests/fixtures/mock-trajectory.json').read_bytes())
PY
bash "$project_dir/run-isaac-python.sh" "$project_dir/import-ur5e.py"
python3 "$project_dir/check-dod.py" "$project_dir/ur5e-import-dod.json"
