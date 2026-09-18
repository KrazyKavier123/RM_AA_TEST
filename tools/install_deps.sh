#!/usr/bin/env bash
set -euo pipefail
ros_distro="${ROS_DISTRO:-humble}"
sudo apt-get update
sudo apt-get install -y build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-yaml python3-opencv libopencv-dev nlohmann-json3-dev \
  "ros-${ros_distro}-cv-bridge" "ros-${ros_distro}-ament-cmake-gtest" \
  libeigen3-dev "ros-${ros_distro}-eigen3-cmake-module" "ros-${ros_distro}-tf2-ros" \
  "ros-${ros_distro}-nav-msgs" "ros-${ros_distro}-trajectory-msgs"
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ros_distro"
