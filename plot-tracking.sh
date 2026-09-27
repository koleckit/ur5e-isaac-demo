#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir="$project_dir"
analysis_venv="$workspace_dir/work/ur-demo-analysis-venv"
python3 "$project_dir/verify-tracking.py"
if [[ ! -x "$analysis_venv/bin/python" ]]; then
  python3 -m venv "$analysis_venv"
  "$analysis_venv/bin/python" -m pip install -r "$project_dir/analysis-requirements.txt"
fi
exec "$analysis_venv/bin/python" "$project_dir/plot-tracking.py"
