# px4_control

PX4 / MAVROS 的 Offboard 控制包。里面有两代东西：

| 节点 | 风格 | 说明 |
|---|---|---|
| `offboard_takeoff` | C++ 状态机 | 起飞悬停示例 |
| `offboard_position_control` | C++ 状态机 | 位置控制示例 |
| `offboard_orbit` | C++ 状态机 | 起飞 / 悬停 / 相对平移 / 环绕 / 自动降落 |
| **`offboard_bt`** | **行为树** | 只提供飞行原语，任务流程写在 `behavior_trees/*.xml` |
| `aruco_down_cam.py` | 视觉 | 下视相机 ArUco 检测与位姿解算 |

前面三个节点的流程写死在 `switch (flight_state_)` 里，每改一次任务都要重编译；
`offboard_bt` 把流程挪到 XML：**改任务只改 XML，不重编译**。

> 🎬 **先看这个：[行为树可视化 docs/bt_visualizer.html](../../docs/bt_visualizer.html)**
> 用浏览器直接打开（单文件、无需联网）。可以一帧一帧看这棵树怎么跑、看
> `Sequence` 与 `ReactiveSequence` 的实测差别、看行为树和 `switch` 状态机到底哪里解耦了。
> **建议先看它，再回来看代码。**

---

## 0. 行为树到底是什么（30 秒版）

行为树 = 把任务写成**一棵树**，由固定频率的 `tick` 驱动。每个节点被 tick 时返回三态之一：

| 返回 | 含义 |
|---|---|
| `SUCCESS` | 做完了 |
| `FAILURE` | 失败 / 做不了 |
| `RUNNING` | 还没做完，**下个 tick 接着做** |

- **控制节点**（`Sequence`/`Fallback`/`Reactive*`）决定"先做谁、后做谁"，自己不干活；
- **叶子节点**（`Takeoff`/`Orbit`）只会"看一眼现状 → 写一次目标 → 汇报状态"，**禁止 while/sleep**。

`Sequence` **有记忆**（记住走到第几个孩子），`ReactiveSequence` 没有（每帧重新评估条件）。
**要持续监控的条件必须用 `Reactive*`**，这是最容易踩的坑，实测数据见可视化文件第 4 节。

为什么它和状态机不同：状态机把"做到第几步 + 每步怎么做 + 什么时候进下一步 + 出异常怎么办"
全塞进同一个 `switch`；行为树把这四件事分别交给**顺序节点、叶子节点、XML 结构、另一棵子树**。
加一个新需求时，状态机要改多处 C++ 并重编译，行为树通常只改 XML。

---

## 1. 依赖

行为树库用的是 **BehaviorTree.CPP v3**（ROS Noetic 官方源里有）：

```bash
sudo apt install ros-noetic-behaviortree-cpp-v3
```

> 装不上（比如没有 sudo）时可以手工解包，不用 root，CMake 一样能找到：
>
> ```bash
> mkdir -p ~/bt_v3 && cd ~/bt_v3
> apt-get download ros-noetic-behaviortree-cpp-v3
> dpkg -x ros-noetic-behaviortree-cpp-v3_*.deb root
> # 编译前把解出来的 prefix 加进 CMAKE_PREFIX_PATH：
> export CMAKE_PREFIX_PATH=~/bt_v3/root/opt/ros/noetic:$CMAKE_PREFIX_PATH
> ```
>
> 运行时还要让动态链接器找到 `libbehaviortree_cpp_v3.so`：
> `export LD_LIBRARY_PATH=~/bt_v3/root/opt/ros/noetic/lib:$LD_LIBRARY_PATH`

## 2. 编译与运行

```bash
cd ~/catkin_ws && catkin_make          # 或者 bash scripts/build.sh
roslaunch px4_control offboard_bt.launch
```

等价的 rosrun 写法：

```bash
rosrun px4_control offboard_bt
rosrun px4_control offboard_bt _tree_file:=$(rospack find px4_control)/behavior_trees/aruco_search.xml
rosrun px4_control offboard_bt _log_transitions:=true      # 打印每次节点状态跳变，调试用
```

跑之前仿真 + mavros 要已经起来（`bash scripts/run_sim.sh`）。

## 3. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `~tree_file` | `<pkg>/behavior_trees/orbit_mission.xml` | 行为树 XML 路径 |
| `~tick_rate` | 20 | 行为树 tick 频率 Hz |
| `~stream_rate` | 50 | setpoint 发布频率 Hz，**必须 > 2** |
| `~prestream_time` | 2.0 | 切 OFFBOARD 前的预推流秒数 |
| `~watchdog` | true | 飞行中丢了 OFFBOARD 是否自动抢回 |
| `~exit_on_complete` | true | 任务结束后是否退出进程 |
| `~log_transitions` | false | 打印节点状态跳变 |

## 4. 架构：为什么不是一个 `while` 循环

```
 行为树层   behavior_trees/*.xml         ← 任务流程（改这里不用重编译）
    ↓ 标签名对应
 原语节点层 src/bt_nodes.cpp             ← 起飞/平移/环绕/降落/条件判断
    ↓ 只写 setpoint
 实时层     SetpointStreamer（独立线程）  ← 固定 50Hz 发 /mavros/setpoint_position/local
 安全层     watchdog（主循环里）          ← 被 RC/超时踢出 OFFBOARD 时抢回来
```

```
 +------------------- 主线程 -------------------+      +------- 推流线程 -------+
 | ros::spinOnce()  -> 更新 DroneState           |      | 50Hz 读 setpoint      |
 | tree.tickRoot()  -> 跑行为树, 更新 setpoint  ---+----> | publish 到 mavros     |
 +-----------------------------------------------+      +-----------------------+
```

三个关键设计，改代码前请先看：

1. **推流必须在独立线程。** PX4 要求 OFFBOARD 期间 setpoint 频率 > 2Hz，断流直接触发
   failsafe。而行为树 tick 里会调用 `arm` / `set_mode` 这类**阻塞服务**，挤在一个循环里
   会把发布周期一起拖垮。所以推流线程只管发，行为树只管想。
2. **安全层放在树外面（`watchdog()`）。** "飞行中被 RC 接管"这种异常要立刻抢回
   OFFBOARD，但又不能打断行为树的执行进度（否则 Sequence 会从头再来一遍、重新起飞）。
   所以任务逻辑进树，模式守护留在主循环。进 `AUTO.LAND` 时 `Land` 节点会把
   `ctx.offboard_recover` 置 false，防止看门狗和降落流程打架。
3. **坐标系约定**：全部是 MAVROS 本地 ENU（x 东 / y 北 / z 上，yaw 逆时针为正），
   mavros 内部自己转 PX4 的 NED。`offboard_bt` 在飞控连上时记录一次**起飞点**
   `origin`，`Takeoff` / `GotoRelative` / `Orbit` 的坐标都以它为原点、z 相对地面。
   `Orbit` 的圆心固定是起飞点，和航点无关，便于预期轨迹。

## 5. 节点清单

XML 里能用的标签，端口就是属性。条件节点只读、瞬时返回；动作节点跨 tick 维护状态。

### 条件节点

| 标签 | 端口 | 说明 |
|---|---|---|
| `IsFcuConnected` | — | MAVROS 是否连上飞控 |
| `IsArmed` | — | 是否已解锁 |
| `IsOffboard` | — | 是否处于 OFFBOARD |
| `IsAtSetpoint` | `tol=0.3` | 当前位姿是否已到位（对比当前 setpoint） |
| `IsTargetVisible` | `timeout=0.5` | `/aruco_down_cam/pose` 是否还有新鲜数据 |

### 动作节点

| 标签 | 端口 | 说明 |
|---|---|---|
| `EnsureOffboard` | `timeout=10` | 一定要进 OFFBOARD：没进去就每秒重试，超时 FAILURE |
| `EnsureArmed` | `timeout=10` | 一定要解锁：同上 |
| `Arm` / `Disarm` | — | 一次性解锁 / 上锁 |
| `Takeoff` | `height=2.0`, `tol=0.25`, `timeout=30` | 起飞到离地 `height`（相对起飞点地面） |
| `Hover` | `duration=3.0` | 原地保持当前 setpoint |
| `GotoRelative` | `x=0`,`y=0`,`z=0`, `yaw`(可选), `tol=0.3`, `timeout=20` | 相对起飞点平移；不写 `yaw` 就机头朝运动方向 |
| `Orbit` | `radius=3`,`speed=1`,`laps=1`, `clockwise=false`, `face_tangent=true`, `z`(可选) | 以起飞点为圆心绕圈；不写 `z` 就保持当前高度 |
| `Land` | `z_threshold=0.15`, `timeout=60` | 请求 `AUTO.LAND` 并等落地 |

BehaviorTree.CPP 自带的结构控制节点也都能用：`Sequence` / `Fallback` /
`ReactiveSequence` / `ReactiveFallback` / `Parallel` / `IfThenElse` / `WhileDoElse` /
`Switch`，装饰器 `Inverter` / `ForceSuccess` / `RetryUntilSuccessful` / `Repeat` /
`Timeout` / `Delay` / `SubTree`。

## 6. 怎么改任务

**只改 XML**。`behavior_trees/orbit_mission.xml` 是最小示例：

```xml
<Sequence name="mission">
  <EnsureOffboard timeout="10"/>
  <EnsureArmed    timeout="10"/>
  <Takeoff height="2.0"/>
  <Hover duration="3.0"/>
  <GotoRelative x="2.0" y="0.0" z="2.0"/>
  <Orbit radius="3.0" speed="1.0" laps="1.0"/>
  <Land/>
</Sequence>
```

`behavior_trees/aruco_search.xml` 演示了行为树相对状态机真正的优势 —— **反应式**：
`ReactiveFallback` 每个 tick 重新判断"目标还在不在"，目标一消失就立刻打断悬停、
自动回到巡逻分支，不需要在 C++ 里为每种组合写状态跳转。

⚠️ **但里面的 `ReactiveSequence` 不能换成 `Sequence`。** `Sequence` 有记忆：它推进到
`Hover` 之后就不会再回头 tick `IsTargetVisible`，结果是"目标丢了也发现不了"，
会硬把 5 秒悬停做完。实测（真实 BehaviorTree.CPP 3.8.6）：

| 观察分支内层用 | 目标丢失后多久离开悬停 |
|---|---|
| `Sequence` | 5.00 s（等悬停自然跑完） |
| `ReactiveSequence` | 0.05 s（下一帧就 halt） |

口诀：**要持续监控的条件 → `ReactiveSequence`/`ReactiveFallback`；
一步一步做完的流程 → `Sequence`/`Fallback`。**

还有一个特性要知道：`Tree::tickRoot()` 在根节点返回 `SUCCESS`/`FAILURE` 后会把根重置为
`IDLE`，所以**任务结束后继续 tick 就是整棵树从头重跑**（飞机再起飞一次）。
`offboard_bt` 的主循环一到终态就退出，正是为了这个。

树的行为不符合预期时，开 `_log_transitions:=true` 看状态跳变；
或者把 `docs/bt_visualizer.html` 里的树改成你的流程，先在浏览器里把它们跑一遍。

## 7. 怎么加自己的节点

在 `src/bt_nodes.cpp` 里照着抄一个，然后在文件末尾 `registerNodes()` 加一行即可。
不需要动 `offboard_bt.cpp`，也不需要动 CMake。

条件节点（必须瞬时返回，禁止服务调用、禁止 sleep）：

```cpp
class BatteryOk : public BT::ConditionNode
{
public:
  BatteryOk(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_volt", 14.0, "最低电压 V")};
  }

  BT::NodeStatus tick() override
  {
    double min_volt = 14.0;
    getInput("min_volt", min_volt);
    // ... 读 ctx_->state 或自己订阅的话题
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};
```

动作节点：跨 tick 的活儿继承 `BT::StatefulActionNode`，用
`onStart()`（返回 RUNNING 表示要接着做）/ `onRunning()` / `onHalted()`；
一个 tick 能办完的（如 `Arm`）继承 `BT::SyncActionNode`，只写 `tick()`。

在树里用的坐标/状态都从 `ctx_` 拿：`ctx_->state`（飞控状态/位姿/视觉目标）、
`ctx_->streamer->set(x, y, z, yaw)`（下发指令）、`ctx_->origin_x/y/z`（起飞点）、
`ctx_->requestOffboard()` / `ctx_->setArmed()`。

> 想接视觉闭环（比如"飞到 ArUco 正上方"）：`/aruco_down_cam/pose` 在
> `camera_optical_frame` 下，要进 ENU 得先过 TF，而 FAST-LIO 的 `odom` 和 MAVROS
> 的本地 ENU 原点并不重合。两条路子：① 引入 `tf2_ros::Buffer` 做
> `camera_optical_frame -> base_link` 变换，再叠到当前位姿上算相对偏移；
> ② 干脆让视觉节点直接输出"机体坐标系下的水平偏差"，
> 新增节点只做速度/位置偏置，避免踩 TF 帧不一致的坑。推荐 ②。

## 8. 调试小贴士

- 行为树可视化：`BehaviorTree.CPP` 自带 `BT::PublisherZMQ`，配上
  [Groot](https://github.com/BehaviorTree/Groot) 可以实时看树的高亮。
- 修改 XML 后不用重编译，直接重启节点即可。
- 如果日志里中文变成 `?????`，那是 rosconsole(log4cxx) 在当前 locale 下的字符集转换
  问题，与代码无关 —— 把 `ROS_INFO/WARN` 的内容改成 ASCII 就不会出现。
