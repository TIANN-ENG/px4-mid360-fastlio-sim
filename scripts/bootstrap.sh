#!/usr/bin/env bash
# 拉取第三方依赖 + 准备 PX4 + 应用 overlay
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_REMOTE="${PX4_REMOTE:-https://github.com/PX4/PX4-Autopilot.git}"

echo "==> [1/4] 拉取第三方 ROS 包（vcstool）"
if ! command -v vcs >/dev/null 2>&1; then
  echo "缺少 vcstool，请先执行： pip3 install --user vcstool"
  exit 1
fi
vcs import "$REPO_DIR/src" < "$REPO_DIR/deps.repos"

echo "==> [2/4] 准备 PX4-Autopilot：$PX4_ROOT"
if [ ! -d "$PX4_ROOT/.git" ]; then
  git clone --recursive "$PX4_REMOTE" "$PX4_ROOT"
else
  echo "    已存在，更新子模块"
  git -C "$PX4_ROOT" submodule update --init --recursive
fi

echo "==> [3/4] 应用 PX4 overlay"
PATCH="$REPO_DIR/px4_overlay/patches/px4-iris-mid360.patch"
if git -C "$PX4_ROOT" apply --check "$PATCH" 2>/dev/null; then
  git -C "$PX4_ROOT" apply "$PATCH"
  echo "    补丁已应用"
else
  echo "    !! 补丁未应用：可能已经打过，或上游代码已变动。"
  echo "       请对照 px4_overlay/README.md 手工核对 4 处改动。"
fi
cp "$REPO_DIR/px4_overlay/airframes/10020_gazebo-classic_iris_mid360" \
   "$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes/"
echo "    airframe 10020_gazebo-classic_iris_mid360 已就位"

echo "==> [4/4] 配置环境变量"
bash "$REPO_DIR/scripts/setup_env.sh"

echo
echo "全部就绪。接着执行："
echo "  source ~/.bashrc"
echo "  bash scripts/build.sh"
