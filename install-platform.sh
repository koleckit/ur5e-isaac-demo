#!/usr/bin/env bash
# UR/MoveIt on host, Docker/NVIDIA runtime, Isaac ROS CLI in Docker mode.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$project_dir"
exec > >(tee -a platform-install.log) 2>&1
trap 'printf "FAILED at line %s. See platform-install.log.\n" "$LINENO" >&2' ERR
date -Is
[[ $EUID != 0 ]] || { echo 'Run as normal user with bash; sudo is used internally.' >&2; exit 1; }
. /etc/os-release
[[ ${VERSION_CODENAME:-} == noble && $(dpkg --print-architecture) == amd64 ]]
[[ -f /opt/ros/lyrical/setup.bash ]]
sudo -v
stage_dir=$(mktemp -d)
trap 'rm -rf -- "$stage_dir"' EXIT
backup_dir="$project_dir/platform-config-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
for existing in /etc/apt/sources.list.d/docker.sources /etc/apt/sources.list.d/nvidia-container-toolkit.list /etc/docker/daemon.json; do
  if [[ -f $existing ]]; then sudo cat "$existing" > "$backup_dir/$(basename "$existing")"; fi
done
curl -fsSL --retry 3 https://download.docker.com/linux/ubuntu/gpg -o "$stage_dir/docker.asc"
sudo install -d -m 0755 /etc/apt/keyrings
sudo install -m 0644 "$stage_dir/docker.asc" /etc/apt/keyrings/docker.asc
cat > "$stage_dir/docker.sources" <<'EOF'
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo install -m 0644 "$stage_dir/docker.sources" /etc/apt/sources.list.d/docker.sources
curl -fsSL --retry 3 https://nvidia.github.io/libnvidia-container/gpgkey -o "$stage_dir/container-key.asc"
gpg --batch --yes --dearmor -o "$stage_dir/container-key.gpg" "$stage_dir/container-key.asc"
sudo install -m 0644 "$stage_dir/container-key.gpg" /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/amd64 /' > "$stage_dir/nvidia-container-toolkit.list"
sudo install -m 0644 "$stage_dir/nvidia-container-toolkit.list" /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
packages=(ros-lyrical-ur ros-lyrical-moveit ros-lyrical-ros2controlcli isaac-ros-cli docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin nvidia-container-toolkit)
apt-get -s install "${packages[@]}" | tee platform-install-dry-run.txt
if grep -q '^Remv ' platform-install-dry-run.txt; then
  echo 'Unexpected package removals; review platform-install-dry-run.txt.' >&2; exit 1
fi
sudo apt-get install -y "${packages[@]}"
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl enable --now docker
sudo systemctl restart docker
sudo usermod -aG docker "$(id -un)"
sudo isaac-ros init docker --yes
sudo docker run --rm hello-world
sudo docker run --rm --gpus all ubuntu:24.04 nvidia-smi | tee docker-gpu-dod.log
sudo docker image inspect hello-world ubuntu:24.04 --format '{{json .RepoDigests}}' > docker-test-image-digests.txt
dpkg-query -W -f='${binary:Package}\t${Version}\n' "${packages[@]}" > platform-package-versions.txt
isaac-ros status | tee isaac-ros-status.txt
bash "$project_dir/test-ur-mock.sh"
echo 'UR mock, Docker GPU and Isaac ROS CLI bootstrap: PASS. Isaac ROS container packages still require activation and their own tests.'
