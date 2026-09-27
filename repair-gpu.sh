#!/usr/bin/env bash
# Run in a normal host terminal; this session cannot elevate privileges.
set -euo pipefail
cd -- "$(dirname -- "$0")"
exec > >(tee -a gpu-repair.log) 2>&1
date -Is
if awk '/NoNewPrivs:/ {exit ($2 == 1 ? 0 : 1)}' /proc/self/status; then
  echo 'Blocked by NoNewPrivs. Run in a normal host terminal.' >&2
  exit 1
fi
kernel_release=$(uname -r)
sudo apt-get update
sudo apt-get install "linux-modules-nvidia-595-open-${kernel_release}" linux-modules-nvidia-595-open-generic-hwe-24.04 nvidia-driver-595-open
echo 'Installation finished. Reboot the host, then run nvidia-smi and audit.sh.'
# No automatic reboot: preserve open work.
