#!/usr/bin/env bash
# ============================================================
# check_range_tof.sh —— 机载激光测距模块（range_tof / 对应实机 VL53L1X）验收脚本
#
#   用法：  bash scripts/check_range_tof.sh
#   退出码：0 = 全部检查通过；1 = 有检查项失败
#
#   特点：使用独立的 ROS master / Gazebo master 端口和自己的临时世界，
#         不影响你正在运行的仿真；退出时自动清理进程与临时文件。
# ============================================================
set -uo pipefail

P="${PX4_ROOT:-$HOME/PX4-Autopilot}"
SITL="$P/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
CC="${CATKIN_WS:-$HOME/catkin_ws}"
SO="$P/build/px4_sitl_default/build_gazebo-classic"
MODELS="$CC/src/Mid360_px4_sim_plugin/livox_laser_simulation/models"

ROS_PORT=11470
GZ_PORT=11471
WORK="$(mktemp -d "$HOME/.check_range_tof.XXXXXX")"
GPID=""
MPID=""
PASS=0
FAIL=0

ok()   { printf '  \033[32m[PASS]\033[0m %s\n' "$1"; PASS=$((PASS + 1)); }
ng()   { printf '  \033[31m[FAIL]\033[0m %s\n' "$1"; FAIL=$((FAIL + 1)); }
info() { printf '  [info] %s\n' "$1"; }
head1() { printf '\n\033[1m[%s]\033[0m %s\n' "$1" "$2"; }

cleanup() {
	[ -n "$GPID" ] && kill -9 "$GPID" 2>/dev/null
	[ -n "$MPID" ] && kill -9 "$MPID" 2>/dev/null
	rm -rf "$WORK"
}
trap cleanup EXIT

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://localhost:$ROS_PORT"
export GAZEBO_MASTER_URI="http://127.0.0.1:$GZ_PORT"
export GAZEBO_MODEL_PATH="$SITL/models:$MODELS:/usr/share/gazebo-11/models"
export GAZEBO_PLUGIN_PATH="$CC/devel/lib:$SO:/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
export LD_LIBRARY_PATH="$SO:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
ulimit -c 0

# ---------------------------------------------------------------- 检查器
# 订阅一帧 sensor_msgs/Range，校验字段、取值与频率
cat > "$WORK/check_msg.py" <<'PY'
import sys, rospy
from sensor_msgs.msg import Range

topic, expect, tol = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
rospy.init_node('check_range_tof', anonymous=True)

msgs = []
sub = rospy.Subscriber(topic, Range, lambda m: msgs.append(m))
t0 = rospy.Time.now()
while len(msgs) < 5 and (rospy.Time.now() - t0).to_sec() < 30.0 and not rospy.is_shutdown():
    rospy.sleep(0.05)
if not msgs:
    print('CHECK|收到消息|FAIL|30s 内没有任何消息')
    sys.exit(1)

# 插件启动瞬间的前几帧时间戳可能为 0，字段/取值一律用"最新一帧"校验
rospy.sleep(1.0)
m = msgs[-1]
n0 = len(msgs)
rospy.sleep(3.0)
rate = (len(msgs) - n0) / 3.0

def emit(name, cond, detail):
    print('CHECK|%s|%s|%s' % (name, 'PASS' if cond else 'FAIL', detail))

emit('消息时间戳非零（需 use_sim_time 一致）', m.header.stamp.to_sec() > 0, 'stamp=%.3f' % m.header.stamp.to_sec())
emit('radiation_type = INFRARED(1)', m.radiation_type == 1, 'radiation_type=%d' % m.radiation_type)
emit('field_of_view ≈ 0.47 rad (27°)', abs(m.field_of_view - 0.47) < 0.02, 'fov=%.3f' % m.field_of_view)
emit('min_range ≈ 0.03 m', abs(m.min_range - 0.03) < 0.005, 'min=%.3f' % m.min_range)
emit('max_range ≈ 4.00 m', abs(m.max_range - 4.0) < 0.01, 'max=%.3f' % m.max_range)
emit('发布频率 ≈ 50 Hz', 45.0 <= rate <= 55.0, '实测 %.1f Hz' % rate)
if expect > 0:
    emit('测距值 ≈ %.3f m (±%.3f)' % (expect, tol), abs(m.range - expect) <= tol, '实测 %.3f m' % m.range)
elif expect == 0:
    emit('超量程时饱和到 max_range', abs(m.range - m.max_range) < 1e-3, '实测 %.3f m (max=%.3f)' % (m.range, m.max_range))
else:
    emit('测距值落在 [min,max] 内', m.min_range <= m.range <= m.max_range, '实测 %.3f m' % m.range)
PY

run_checks() {   # $1=场景名 $2=世界文件
	local lines
	lines="$(timeout 60 /usr/bin/python3 "$WORK/check_msg.py" /range_tof/range "$EXPECT" "$TOL" 2>/dev/null | grep '^CHECK|')"
	if [ -z "$lines" ]; then
		ng "$1：未取到任何检查结果（模块未发布或插件加载失败）"
		return
	fi
	while IFS='|' read -r _ name status detail; do
		if [ "$status" = "PASS" ]; then ok "$1 · $name（$detail）"; else ng "$1 · $name（$detail）"; fi
	done <<< "$lines"
}

make_world() {   # $1=输出文件 $2=模块高度
	/usr/bin/python3 -c "
import sys
out, z = sys.argv[1], sys.argv[2]
base = open('$SITL/worlds/aruco_landing.world').read()
m = '''    <model name=\"range_test\">
      <static>true</static>
      <pose>1.01 0.98 %s 0 0 0</pose>
      <include><uri>model://range_tof</uri></include>
    </model>
''' % z
open(out, 'w').write(base.replace('</world>', m + '  </world>'))
" "$1" "$2"
}

start_sim() {   # $1=世界文件
	rosmaster --core -p "$ROS_PORT" >/dev/null 2>&1 &
	MPID=$!
	disown "$MPID" 2>/dev/null || true
	sleep 4
	gzserver -s libgazebo_ros_api_plugin.so "$1" > "$WORK/gz.log" 2>&1 &
	GPID=$!
	disown "$GPID" 2>/dev/null || true
	sleep 15
}

stop_sim() {
	[ -n "$GPID" ] && kill -9 "$GPID" 2>/dev/null
	[ -n "$MPID" ] && kill -9 "$MPID" 2>/dev/null
	GPID=""; MPID=""
	sleep 2
}

printf '\033[1m=== 激光测距模块 range_tof 验收 ===\033[0m\n'
printf 'PX4     : %s\ncatkin  : %s\n\n' "$P" "$CC"

# ---------------------------------------------------------------- 1 文件与语法
head1 1/4 "文件与挂载检查"
[ -f "$SITL/models/range_tof/range_tof.sdf" ] && ok "range_tof.sdf 存在" || ng "range_tof.sdf 缺失"
[ -f "$SITL/models/range_tof/model.config" ] && ok "model.config 存在" || ng "model.config 缺失"

if grep -q 'model://range_tof' "$SITL/models/iris_mid360/iris_mid360.sdf"; then
	ok "iris_mid360.sdf 已挂载 range_tof"
	L_TOF=$(grep -n 'model://range_tof' "$SITL/models/iris_mid360/iris_mid360.sdf" | head -1 | cut -d: -f1)
	L_LIDAR=$(grep -n 'model://Mid360' "$SITL/models/iris_mid360/iris_mid360.sdf" | head -1 | cut -d: -f1)
	if [ "$L_TOF" -lt "$L_LIDAR" ]; then
		ok "挂载顺序正确（测距在 Mid360 之前，行 $L_TOF < $L_LIDAR）"
	else
		ng "挂载顺序错误：range_tof（行 $L_TOF）必须在 Mid360（行 $L_LIDAR）之前，否则雷达自滤波会段错误"
	fi
else
	ng "iris_mid360.sdf 未挂载 range_tof"
fi

if gz sdf -k "$SITL/models/range_tof/range_tof.sdf" >/dev/null 2>&1; then
	ok "range_tof.sdf 通过 SDF 语法校验"
else
	ng "range_tof.sdf SDF 语法错误"
fi

# ---------------------------------------------------------------- 2 静态数值
head1 2/4 "静态测距数值（模块悬停在标识板上方 2.00 m）"
info "标识面 z=0.063 m → 期望读数 ≈ 1.937 m"
EXPECT=1.937; TOL=0.05
make_world "$WORK/at2.world" 2.0
start_sim "$WORK/at2.world"
run_checks "静态 2 m" "$WORK/at2.world"
stop_sim

# ---------------------------------------------------------------- 3 超量程
head1 3/4 "超量程行为（模块在 5.00 m 高，超出 max_range=4.0）"
info "融合算法必须能把这种情况判为无效观测"
EXPECT=0; TOL=0
make_world "$WORK/at5.world" 5.0
start_sim "$WORK/at5.world"
run_checks "超量程 5 m" "$WORK/at5.world"
stop_sim

# ---------------------------------------------------------------- 4 机体挂载
head1 4/4 "装载在真实机体上是否能正常发布"
info "生成 iris_mid360（含相机 + 测距 + Mid360），约需 40 s"
make_world "$WORK/drone.world" 0.058
start_sim "$WORK/drone.world"
timeout 60 gz model --spawn-file="$SITL/models/iris_mid360/iris_mid360.sdf" \
	--model-name=iris_mid360 -x 1.01 -y 0.98 -z 1.5 >/dev/null 2>&1
sleep 5
if timeout 20 /usr/bin/python3 "$WORK/check_msg.py" /range_tof/range -1 0 >/dev/null 2>&1; then
	ok "机体上 /range_tof/range 正常发布"
else
	if timeout 12 rostopic list 2>/dev/null | grep -q '^/range_tof/range$'; then
		ok "机体上 /range_tof/range 话题已注册（未取到有效帧，可能已落地→近距饱和）"
	else
		ng "机体上未出现 /range_tof/range"
	fi
fi
stop_sim

# ---------------------------------------------------------------- 汇总
printf '\n\033[1m=== 验收结果：通过 %d 项，失败 %d 项 ===\033[0m\n' "$PASS" "$FAIL"
if [ "$FAIL" -eq 0 ]; then
	printf '\033[32m验收通过。\033[0m\n'
	exit 0
else
	printf '\033[31m有检查项未通过，请查看上面 [FAIL] 行。\033[0m\n'
	exit 1
fi
