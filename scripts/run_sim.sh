#!/usr/bin/env bash
# 一键启动仿真链路
#   roscore + use_sim_time  ->  PX4/Gazebo  ->  MAVROS  ->  FAST-LIO
#
# 用法：
#   bash scripts/run_sim.sh            # tmux 一次拉起全部（推荐）
#   bash scripts/run_sim.sh core       # 终端1：roscore + 锁定 /use_sim_time
#   bash scripts/run_sim.sh px4        # 终端2：PX4 SITL + Gazebo
#   bash scripts/run_sim.sh mavros     # 终端3：MAVROS + 静态 TF
#   bash scripts/run_sim.sh lio        # 终端4：FAST-LIO + RViz
#
# 注意：/use_sim_time 必须【先于】gzserver 设置，否则 Gazebo 侧会锁在墙钟上，
#       FAST-LIO 永远配不上 IMU，TF 里就不会出现 odom->base_link。
#       详见 docs/pitfalls.md
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
WS="$REPO_DIR"
PX4_MODEL="${PX4_MODEL:-iris_mid360}"
PX4_WORLD="${PX4_WORLD:-aruco_landing}"
SESSION="${SESSION:-mid360_sim}"

CMD_PX4="cd '$PX4_ROOT' && make px4_sitl gazebo-classic_${PX4_MODEL}__${PX4_WORLD}"
CMD_MAVROS="source '$WS/devel/setup.bash' && roslaunch iris_description mavros_tf.launch"
CMD_LIO="source '$WS/devel/setup.bash' && roslaunch fast_lio mapping_mid360.launch"
#当换成实机时需要在fastlio下修改mapping_mid360.launch的指向config文件修改为实机的config —— 实机用 mid360.yaml，仿真用 my_mid360.yaml
usage() { sed -n '2,14p' "$0"; }

case "${1:-all}" in
  core)
    echo "==> 启动 roscore"
    roscore &
    sleep 4
    echo "==> 锁定仿真时钟（必须在 gzserver 之前）"
    rosparam set use_sim_time true
    echo "    /use_sim_time = $(rosparam get /use_sim_time)"
    wait
    ;;
  px4)     exec bash -lc "$CMD_PX4" ;;
  mavros)  exec bash -lc "$CMD_MAVROS" ;;
  lio)     exec bash -lc "$CMD_LIO" ;;
  all)
    if ! command -v tmux >/dev/null 2>&1; then
      echo "未安装 tmux（sudo apt install tmux）。请改用 4 个终端："
      usage
      exit 1
    fi
    tmux kill-session -t "$SESSION" 2>/dev/null || true

    tmux new-session -d -s "$SESSION" -n core
    tmux send-keys -t "${SESSION}:core" "bash '$REPO_DIR/scripts/run_sim.sh' core" C-m
    echo "==> [1/4] roscore + use_sim_time"
    sleep 6

    tmux new-window -t "$SESSION" -n px4
    tmux send-keys -t "${SESSION}:px4" "$CMD_PX4" C-m
    echo "==> [2/4] PX4 + Gazebo（等待 25s）"
    sleep 25

    tmux new-window -t "$SESSION" -n mavros
    tmux send-keys -t "${SESSION}:mavros" "$CMD_MAVROS" C-m
    echo "==> [3/4] MAVROS"
    sleep 8

    tmux new-window -t "$SESSION" -n lio
    tmux send-keys -t "${SESSION}:lio" "$CMD_LIO" C-m
    echo "==> [4/4] FAST-LIO"

    echo
    echo "全部启动。进入 tmux（Ctrl-b d 脱离，脚本 scripts/kill_sim.sh 结束）"
    echo "自检： bash scripts/check_env.sh"
    tmux select-window -t "${SESSION}:lio"
    tmux attach -t "$SESSION"
    ;;
  *) usage; exit 1 ;;
esac
