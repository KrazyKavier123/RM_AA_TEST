#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "ROS2 setup missing: /opt/ros/${ros_distro}/setup.bash" >&2; exit 1
fi
set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u
cd "$workspace_dir"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "$@"
