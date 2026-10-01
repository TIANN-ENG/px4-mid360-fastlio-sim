#!/usr/bin/env bash
# 环境自检：进程 / 模型路径 / overlay / 时钟 / 话题 / TF
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
hr() { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }

hr "1. 相关进程"
ps -ef | grep -E "bin/px4|gzserver|gzclient|mavros_node|fastlio_mapping|rosmaster" | grep -v grep || echo "  （无）"

hr "2. gazebo 模型路径"
echo "  GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
old_IFS="$IFS"; IFS=":"; _paths=(${GAZEBO_MODEL_PATH:-}); IFS="$old_IFS"
for m in Mid360 iris_mid360; do
  found=""
  for p in "${_paths[@]}"; do [ -d "$p/$m" ] && found="$p/$m"; done
  printf "  %-12s -> %s\n" "$m" "${found:-未找到 ⚠️}"
done

hr "3. PX4 overlay 是否已应用"
AF="$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_iris_mid360"
[ -f "$AF" ] && echo "  ✅ airframe 10020 存在" || echo "  ❌ 缺 airframe 10020"
grep -q "iris_mid360" "$PX4_ROOT/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake" 2>/dev/null \
  && echo "  ✅ sitl_targets 已注册 iris_mid360" || echo "  ❌ sitl_targets 未注册"
grep -q "10020_gazebo-classic_iris_mid360" "$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" 2>/dev/null \
  && echo "  ✅ airframes/CMakeLists.txt 已注册" || echo "  ❌ airframes/CMakeLists.txt 未注册"

hr "4. 时钟（最容易出问题的一项）"
if rostopic list >/dev/null 2>&1; then
  echo "  /use_sim_time = $(rosparam get /use_sim_time 2>/dev/null)"
  echo -n "  /livox/lidar      stamp: "; timeout 5 rostopic echo -n1 /livox/lidar/header/stamp 2>/dev/null | tr '\n' ' '; echo
  echo -n "  /mavros/imu/data  stamp: "; timeout 5 rostopic echo -n1 /mavros/imu/data/header/stamp 2>/dev/null | tr '\n' ' '; echo
  echo "  两者数量级必须一致；一个 17 亿、一个几十 => gzserver 没进仿真时钟模式"
else
  echo "  roscore 未运行"
fi

hr "5. 关键话题频率"
for t in /livox/lidar /mavros/imu/data /Odometry; do
  printf "  %-20s : " "$t"
  timeout 8 rostopic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据 ⚠️"
done

hr "6. TF: odom -> base_link"
timeout 6 rosrun tf tf_echo odom base_link 2>&1 | head -10

hr "7. 其它"
echo "  conda 环境: ${CONDA_DEFAULT_ENV:-无}"
[ -n "${CONDA_DEFAULT_ENV:-}" ] && echo "    ⚠️ 先 conda deactivate，否则 roslaunch 可能找不到包"
