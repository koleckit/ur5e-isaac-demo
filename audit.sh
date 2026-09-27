#!/usr/bin/env bash
set -u
cd -- "$(dirname -- "$0")"
exec > >(tee audit-latest.txt) 2>&1
run() { printf '\n$'; printf ' %q' "$@"; printf '\n'; "$@"; printf '[exit=%s]\n' "$?"; }
run date -Is
run cat /etc/os-release
run uname -a
run lscpu
run free -h
run df -h . /dev/shm
run lspci -nnk -d 10de:
run nvidia-smi
run cat /proc/driver/nvidia/version
run mokutil --sb-state
run find /lib/modules -name 'nvidia*.ko*'
run bash -c 'dpkg-query -W -f="${db:Status-Abbrev} ${binary:Package} ${Version}\n" 2>/dev/null | rg "^ii .*nvidia|^ii .*cuda|^ii .*docker|^ii .*ros-lyrical"'
run bash -c 'for t in nvcc docker nvidia-ctk ros2 isaac-ros; do command -v "$t" || printf "%s: absent from PATH\n" "$t"; done'
run ip -br address
run ip route
run cat /sys/class/net/enp9s0/speed
run cat /sys/class/net/enp9s0/carrier
run bash -c 'rg "NoNewPrivs|Uid|CapEff" /proc/self/status'
run cat "${ISAAC_SIM_DIR:-$HOME/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64}/VERSION"
