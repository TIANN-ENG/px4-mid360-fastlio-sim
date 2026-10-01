# PX4 + Gazebo Classic + Livox Mid-360 + FAST-LIO 仿真建图平台

在 Ubuntu 20.04 上用 **PX4 SITL + Gazebo Classic** 仿真一架挂载 **Livox Mid-360**
的 Iris 无人机，经 **MAVROS** 把仿真 IMU 与雷达点云送入 **FAST-LIO**，
实现实时 LiDAR-惯性建图；并附带 `px4_control` 做 Offboard 飞行控制。
改模型、换世界、调算法都在这条流水线上迭代。

> 个人研究项目，持续迭代中。有问题欢迎开 issue。

## 效果

![建图效果](docs/images/mapping.png)

> 截图/录屏放在 `docs/images/`，把上面这行换成自己的图。

## 系统架构

```
                MAVLink / UDP :14540
  PX4 SITL  ─────────────────────────────►  MAVROS ──/mavros/imu/data──┐
      ▲                                                                 │
      │  lockstep                                       ┌───────────────▼──────────────┐
      │                                                 │          FAST-LIO            │
  Gazebo Classic ──liblivox_laser_simulation.so──►      │  /Odometry                   │
      └─ iris_mid360（Iris 机身 + Mid-360 下倾安装）     │  odom -> base_link TF        │
                                     /livox/lidar       └──────────────────────────────┘
```

## 环境依赖

| 组件 | 版本 / 说明 |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | Noetic（ROS1） |
| 仿真器 | Gazebo Classic 11 |
| 飞控 | PX4-Autopilot（fork 自上游，见 `px4_overlay/`） |
| 通信 | mavros 1.20.1 |
| 建图 | FAST_LIO（GPL-2.0，独立进程） |
| 雷达插件 | [Mid360_px4_sim_plugin](https://github.com/Tfly6/Mid360_px4_sim_plugin) |

## 目录结构

```
~/catkin_ws/                    # 仓库根 = catkin 工作空间
├── src/                        # catkin 源码空间
│   ├── iris_description/       # 本仓库：模型描述 / mavros TF / launch
│   ├── px4_control/            # 本仓库：Offboard 控制示例
│   ├── FAST_LIO/               # 第三方（deps.repos 拉取，仓库内被 .gitignore）
│   ├── Mid360_px4_sim_plugin/  # 第三方（deps.repos 拉取，含雷达插件与模型）
│   └── livox_ros_driver2/      # 第三方（可选，仅真机用）
├── deps.repos                  # 第三方依赖声明（vcstool）
├── px4_overlay/                # 对 PX4 的最小改动（补丁 + airframe）
├── scripts/                    # 一键安装 / 拉取 / 编译 / 启动 / 自检
└── docs/                       # 架构说明与踩坑记录
```

## 快速开始

### 0. 依赖

```bash
sudo apt install git tmux python3-pip python3-vcstool \
                 ros-noetic-mavros ros-noetic-mavros-extras \
                 ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs \
                 ros-noetic-rviz libeigen3-dev libpcl-dev
```

> ⚠️ 装了 conda / anaconda 的话，**每开一个新终端先 `conda deactivate`**。
> conda 的 python 会顶掉 ROS 的，典型症状是 `roslaunch` 报找不到包。

### 1. 拉代码

```bash
git clone https://github.com/TIANN-ENG/px4-mid360-fastlio-sim.git ~/catkin_ws
cd ~/catkin_ws
bash scripts/bootstrap.sh      # vcs import + clone PX4 + 应用 overlay + 写 ~/.bashrc
source ~/.bashrc
```

`bootstrap.sh` 会做四件事：

1. 按 `deps.repos` 把 FAST_LIO、Mid360 插件拉到 `src/`
2. 若 `$HOME/PX4-Autopilot` 不存在则 clone 上游 PX4（含子模块）
3. 给 PX4 打上 `px4_overlay/patches/px4-iris-mid360.patch`，并放入 airframe `10020`
4. 把 ROS / Gazebo / PX4 环境变量写入 `~/.bashrc`

### 2. 编译

```bash
bash scripts/build.sh          # PX4: make px4_sitl_default  +  ROS: catkin_make
```

### 3. 运行

**推荐：一条命令（tmux 四窗口）**

```bash
bash scripts/run_sim.sh
```

**或手动开四个终端**（顺序不能乱）：

```bash
# 终端 1 —— 先起 master 并锁定仿真时钟
bash scripts/run_sim.sh core

# 终端 2 —— 仿真：Gazebo + PX4 + iris_mid360
bash scripts/run_sim.sh px4

# 终端 3 —— MAVROS + 静态 TF
bash scripts/run_sim.sh mavros

# 终端 4 —— FAST-LIO 建图
bash scripts/run_sim.sh lio
```

> **为什么必须这样才能连上 TF？**
> `/use_sim_time` 是节点启动时一次性读取的，运行中再改无效。
> 如果先起 Gazebo 再起 roscore，gzserver 会锁在墙钟时间上，而 MAVROS 用仿真时间，
> FAST-LIO 永远配不上 IMU，`/Odometry` 一条都不出，TF 里就没有 `odom -> base_link`。
> 详见 [docs/pitfalls.md](docs/pitfalls.md)。

### 4. 自检

```bash
bash scripts/check_env.sh
```

单项验证：

```bash
rostopic hz /livox/lidar          # 雷达点云
rostopic hz /mavros/imu/data      # 仿真 IMU
rostopic hz /Odometry             # FAST-LIO 输出（有频率才算成功）
rosrun tf tf_echo odom base_link  # TF 链路
```

### 5. Offboard 控制（可选）

```bash
rosrun px4_control offboard_takeoff
rosrun px4_control offboard_position_control
```

### 6. 结束

```bash
bash scripts/kill_sim.sh          # 加 --all 连 roscore 一起关
```

## 话题与 TF

| 话题 | 类型 | 来源 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Gazebo 插件 `liblivox_laser_simulation.so` |
| `/mavros/imu/data` | `sensor_msgs/Imu` | MAVROS ← PX4 SITL |
| `/Odometry` `/path` `/cloud_registered` `/Laser_map` | — | FAST-LIO |
| `/mavros/local_position/*` `/mavros/state` | — | MAVROS |

TF：`odom -> base_link`（FAST-LIO 发布）+ `base_link -> livox_link`（`mavros_tf.launch`）。

本项目**不发布 `map` 帧**，需要时手动补：

```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
```

另外 Gazebo 会给点云加上模型前缀，`/livox/lidar` 的 `frame_id` 实际是
`Mid360::livox_link`，RViz 里对不上时补：

```bash
rosrun tf static_transform_publisher 0.07 0 0.072 0 0.3925 0 base_link "Mid360::livox_link" 100
```

## 关键参数

| 项 | 值 | 位置 |
|---|---|---|
| Mid-360 安装位姿 | `0.07 0 0.072 0 0.3925 0`（前 0.07m / 上 0.072m / 下倾 0.3925rad） | `src/Mid360_px4_sim_plugin/livox_laser_simulation/models/iris_mid360/iris_mid360.sdf` |
| FAST-LIO 雷达外参 | `extrinsic_T = [0.070, 0.000, 0.072]`，`extrinsic_R = Ry(0.3925)` | `src/FAST_LIO/config/my_mid360.yaml` |
| 仿真雷达配置 | `lidar_type: 4`（插件发 `PointCloud2`）+ `imu_topic: /mavros/imu/data` | 同上 |
| 真机雷达配置 | `lidar_type: 1`（`CustomMsg`）+ `/livox/imu` | `src/FAST_LIO/config/mid360.yaml` |

## 二次开发

- **换世界**：`make px4_sitl gazebo-classic_iris_mid360__warehouse`（注意是双下划线）
- **改雷达安装角**：改 `iris_mid360.sdf` 里的 `<pose>`，同时同步 FAST-LIO 的 `extrinsic_T/R`
- **改算法参数**：`src/FAST_LIO/config/my_mid360.yaml`
- **换机型/加传感器**：在 `Mid360_px4_sim_plugin` 的 models 里加 SDF，并在
  `px4_overlay/patches/` 与 `deps.repos` 中登记
- **同步上游**：`git -C src/FAST_LIO fetch upstream && git rebase upstream/main`

## 上传到 GitHub

首次上传只需在网页做三件事：

1. fork https://github.com/Tfly6/Mid360_px4_sim_plugin/fork
2. fork https://github.com/hku-mars/FAST_LIO/fork
3. 新建 https://github.com/new，名称 `px4-mid360-fastlio-sim`，
   **不要**勾选 Add README / .gitignore / License（本地已有，勾了会导致 push 冲突）

然后一条命令推送全部：

```bash
bash scripts/push_to_github.sh
```

脚本会先校验三个远程仓库是否已存在，再依次推送两个第三方分支
（`px4-sim-fixes`、`mid360-px4-sim`）和主仓库的 `main`。

日常迭代：改完自己的包后 `git add -A && git commit -m "..." && git push`；
如果改的是 `src/FAST_LIO` 或 `src/Mid360_px4_sim_plugin`，需要分别进那两个目录提交并推送。

## FAQ

**Q: `odom -> base_link` 连不上，`/Odometry` 没数据**
A: 99% 是时钟问题，先跑 `bash scripts/check_env.sh` 看第 4 项，见 [docs/pitfalls.md](docs/pitfalls.md)。

**Q: `roslaunch` 报找不到包 / python 报错**
A: `conda deactivate`。

**Q: Gazebo 里没有点云 / `/livox/lidar` 没数据**
A: 确认 `GAZEBO_MODEL_PATH` 里有 `Mid360` 与 `iris_mid360`（`check_env.sh` 第 2 项），
   并且插件已编译出 `devel/lib/liblivox_laser_simulation.so`。

**Q: 卡在 `make px4_sitl` 编译**
A: 第一次编译 PX4 比较久（10~30 分钟），加 `-j` 见 `scripts/build.sh`。

## 致谢

感谢这些优秀的开源工作：

- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) — BSD-3-Clause
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO) — GPL-2.0 · Xu et al., *FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package*
- [Mid360_px4_sim_plugin](https://github.com/Tfly6/Mid360_px4_sim_plugin) — Gazebo Mid-360 雷达插件
- [mavros](https://github.com/mavlink/mavros) · [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) · [Livox-SDK](https://github.com/Livox-SDK)

## License

本仓库中由作者编写的部分采用 **MIT**（见 [LICENSE](LICENSE)）。
通过 `deps.repos` 引用的第三方组件遵循各自原始许可证（FAST_LIO 为 GPL-2.0）。
