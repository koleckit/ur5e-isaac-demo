#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
context_dir="$project_dir/../../work/isaac-docker-context"
mkdir -p "$context_dir"
sg docker -c "docker build --progress=plain -t ur-demo:isaac-ros-5.0 -f '$project_dir/Dockerfile.isaac-ros' '$context_dir'" 2>&1 | tee "$project_dir/cumotion-image-build.log"
