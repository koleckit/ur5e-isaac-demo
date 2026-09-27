#!/usr/bin/env bash
# Rebuild the pinned UR 6.0.0 calibration tool with explicit URCL initialization.
set -eo pipefail
setup_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(dirname -- "$setup_dir")
workspace_dir="$project_dir"
source "$project_dir/ros-env.sh"
build_dir="$workspace_dir/work/ur-calibration-build"
cmake -S "$setup_dir/calibration-tool" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j 4
timeout 30s "$build_dir/calibration_correction_fixed" --ros-args \
  -p robot_ip:=192.168.60.201 -p "output_filename:=$setup_dir/ur5e-calibration.yaml"
python3 - "$setup_dir/ur5e-calibration.yaml" <<'PY'
import sys, yaml, math
from pathlib import Path
kinematics = yaml.safe_load(Path(sys.argv[1]).read_text())['kinematics']
assert kinematics['hash'].startswith('calib_')
for name in ('shoulder','upper_arm','forearm','wrist_1','wrist_2','wrist_3'):
    assert all(math.isfinite(kinematics[name][key]) for key in ('x','y','z','roll','pitch','yaw'))
print('Calibration YAML validated:', kinematics['hash'])
PY
xacro /opt/ros/lyrical/share/ur_description/urdf/ur.urdf.xacro \
  name:=ur5e ur_type:=ur5e "kinematics_params:=$setup_dir/ur5e-calibration.yaml" > "$setup_dir/ur5e-calibrated.urdf"
