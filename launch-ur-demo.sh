#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export UR_DEMO_PHYSICAL_EXECUTE=1
exec bash "$project_dir/launch-physical-preview.sh"
