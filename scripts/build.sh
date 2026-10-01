#!/usr/bin/env bash
# 编译 PX4 SITL 与 ROS 工作空间
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
NPROC="$(nproc)"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "==> [1/2] 编译 PX4 SITL（-j${NPROC}）"
make -C "$PX4_ROOT" px4_sitl_default -j"${NPROC}"

echo "==> [2/2] 编译 ROS 工作空间"
cd "$REPO_DIR"
catkin_make -j"$(( NPROC > 4 ? 4 : NPROC ))"

echo
echo "完成。新终端会通过 ~/.bashrc 自动 source；当前终端请执行："
echo "  source $REPO_DIR/devel/setup.bash"
