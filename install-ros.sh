#!/usr/bin/env bash
# ROS 2 Lyrical on Ubuntu Noble, using the official NVIDIA Buildfarm.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$project_dir"
exec > >(tee -a ros-install.log) 2>&1
trap 'printf "FAILED at line %s; ROS installation is not complete.\n" "$LINENO" >&2' ERR
date -Is
if [[ $EUID == 0 ]]; then
  echo 'Run with bash as your normal user; the script uses sudo when needed.' >&2
  exit 1
fi
. /etc/os-release
[[ ${VERSION_CODENAME:-} == noble && $(dpkg --print-architecture) == amd64 ]] || {
  echo 'This installer requires Ubuntu 24.04 Noble amd64.' >&2; exit 1;
}
if awk '/NoNewPrivs:/ {exit ($2 == 1 ? 0 : 1)}' /proc/self/status; then
  echo 'Privilege elevation is blocked. Run from a normal host terminal.' >&2
  exit 1
fi
sudo -v
sudo apt-get update
sudo apt-get install -y curl ca-certificates gnupg locales
locale charmap | grep -qx UTF-8 || {
  sudo locale-gen en_US.UTF-8
  export LANG=en_US.UTF-8
  export LC_ALL=en_US.UTF-8
}
stage_dir=$(mktemp -d)
trap 'rm -rf -- "$stage_dir"' EXIT
backup_dir="$project_dir/ros-apt-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
for existing in /usr/share/keyrings/nvidia-isaac-ros.gpg /etc/apt/sources.list.d/nvidia-isaac-ros.list /etc/apt/sources.list.d/nvidia-isaac-ros-buildfarm.list; do
  if [[ -f $existing ]]; then cp -- "$existing" "$backup_dir/"; fi
done
curl --fail --location --retry 3 --output "$stage_dir/nvidia-key.asc" https://isaac.download.nvidia.com/isaac-ros/repos.key
gpg --batch --yes --dearmor --output "$stage_dir/nvidia-isaac-ros.gpg" "$stage_dir/nvidia-key.asc"
gpg --show-keys --with-fingerprint "$stage_dir/nvidia-isaac-ros.gpg"
sudo install -m 0644 "$stage_dir/nvidia-isaac-ros.gpg" /usr/share/keyrings/nvidia-isaac-ros.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main' > "$stage_dir/nvidia-isaac-ros.list"
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main' > "$stage_dir/nvidia-isaac-ros-buildfarm.list"
sudo install -m 0644 "$stage_dir/nvidia-isaac-ros.list" /etc/apt/sources.list.d/nvidia-isaac-ros.list
sudo install -m 0644 "$stage_dir/nvidia-isaac-ros-buildfarm.list" /etc/apt/sources.list.d/nvidia-isaac-ros-buildfarm.list
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -qx 'install ok installed'; then
  curl --fail --location --retry 3 --output "$stage_dir/ros2-apt-source.deb" https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb
  sudo dpkg -i "$stage_dir/ros2-apt-source.deb"
fi
sudo apt-get update
apt-cache policy ros-lyrical-ros-base ros-lyrical-desktop | tee ros-candidates.txt
apt-cache policy ros-lyrical-ros-base | grep -Fq 'https://isaac.download.nvidia.com/isaac-ros/ubuntu/main' || {
  echo 'NVIDIA Buildfarm source missing from ros-base policy.' >&2; exit 1;
}
[[ $(apt-cache policy ros-lyrical-ros-base | awk '/Candidate:/ {print $2}') != '(none)' ]] || {
  echo 'No ros-base candidate; stopping.' >&2; exit 1;
}
# Desktop meta-package currently requires unavailable OpenCV/PCL/PySide6
# packages on Noble. The required base + RViz + demos resolve successfully.
ros_packages=(ros-lyrical-ros-base ros-lyrical-rviz2 ros-lyrical-demo-nodes-cpp ros-lyrical-demo-nodes-py python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential)
apt-get -s install "${ros_packages[@]}" | tee ros-install-dry-run.txt
if grep -q '^Remv ' ros-install-dry-run.txt; then
  echo 'Installation would remove existing packages. Review ros-install-dry-run.txt before continuing.' >&2
  exit 1
fi
sudo apt-get install -y "${ros_packages[@]}"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --rosdistro lyrical
dpkg-query -W -f='${binary:Package}\t${Version}\n' 'ros-lyrical-*' > ros-package-versions.txt
bash "$project_dir/test-ros.sh"
echo 'ROS Lyrical installation and pub/sub Definition of Done: PASS.'
