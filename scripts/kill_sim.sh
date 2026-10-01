#!/usr/bin/env bash
# 结束仿真相关进程。加 --all 连 roscore 一起关。
set -uo pipefail

echo "==> 结束仿真进程"
for p in sitl_run px4 gzserver gzclient gazebo; do
  pkill -9 -f "$p" 2>/dev/null && echo "    killed: $p" || true
done

echo "==> 结束算法/中间件进程"
for p in mavros_node fastlio_mapping rviz rqt_gui_py_node; do
  pkill -9 -f "$p" 2>/dev/null && echo "    killed: $p" || true
done

if [ "${1:-}" = "--all" ]; then
  echo "==> 结束 ROS master"
  pkill -9 -f rosmaster 2>/dev/null || true
  pkill -9 -f "rosout"  2>/dev/null || true
fi

echo "完成。"
