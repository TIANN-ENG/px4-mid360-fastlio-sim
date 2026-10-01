#!/usr/bin/env bash
# 安装系统依赖，并把本项目所需的环境变量写入 ~/.bashrc
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"

echo "==> 安装 apt 依赖（需要 sudo）"
sudo apt-get update
sudo apt-get install -y \
  git tmux python3-pip python3-vcstool \
  ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
  ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rqt-gui-py \
  libeigen3-dev libpcl-dev

MARKER_BEGIN="# >>> px4-mid360-fastlio-sim >>>"
MARKER_END="# <<< px4-mid360-fastlio-sim <<<"

if grep -qF "$MARKER_BEGIN" "$HOME/.bashrc"; then
  echo "==> ~/.bashrc 已存在本项目配置块，跳过"
  echo "    （如需更新：删掉 $MARKER_BEGIN ... $MARKER_END 之间内容后重跑）"
else
  echo "==> 写入 ~/.bashrc"
  cat >> "$HOME/.bashrc" <<BASHRC

$MARKER_BEGIN
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${REPO_DIR}/devel/setup.bash" ]; then
    source "${REPO_DIR}/devel/setup.bash"
fi

# Gazebo ROS 插件搜索路径
export GAZEBO_PLUGIN_PATH="\$GAZEBO_PLUGIN_PATH:${REPO_DIR}/devel/lib:/opt/ros/${ROS_DISTRO}/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
# Mid360 / iris_mid360 模型由插件仓库提供，无需再往 PX4 里拷模型
export GAZEBO_MODEL_PATH="\$GAZEBO_MODEL_PATH:${REPO_DIR}/src/Mid360_px4_sim_plugin/livox_laser_simulation/models"

# PX4
export PX4_ROOT="${PX4_ROOT}"
export PX4_SIM_MODEL="iris_mid360"
$MARKER_END
BASHRC
fi

echo "==> 完成。请执行： source ~/.bashrc"
