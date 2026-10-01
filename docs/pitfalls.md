# 踩坑记录

## 1. `odom -> base_link` 连不上，`/Odometry` 完全没有数据

### 现象

- `rosrun tf tf_echo odom base_link` 报 `Frame odom does not exist`
- `rostopic hz /Odometry` 无数据
- `/livox/lidar` 和 `/mavros/imu/data` **各自都有数据**
- `laserMapping` 节点在 `rosnode list` 里存在，但终端**一行日志都没有**
  （连 `No point, skip this scan!` 都没有）

### 根因：两个进程用了不同的时钟

Gazebo 侧和 MAVROS 侧的时间戳差了 9 个数量级：

| 节点 | 发布的话题 | 时间戳 |
|---|---|---|
| `/gazebo`（livox 插件） | `/livox/lidar` | `1790776999`（墙钟） |
| `/mavros` | `/mavros/imu/data` | `8.4`（仿真时间） |

FAST-LIO 的 `sync_packages()` 里有一句硬性判断：

```cpp
if (last_timestamp_imu < lidar_end_time) return false;   // 10 < 1.79e9，永远成立
```

IMU 永远"追不上"雷达时间戳，于是**一个扫描都配不上对**，主循环体从未执行，
`publish_odometry()` 自然没被调用过 —— 而 `odom -> base_link` 的 TF 广播在
`publish_odometry()` 内部（函数局部 `static tf::TransformBroadcaster`），
函数没调用过，`/tf` 发布者甚至不会向 master 注册。

### 为什么会这样

`/use_sim_time` 是 **roscpp 在节点启动时一次性读取**的，运行中修改无效。

Gazebo 的 `libgazebo_ros_api_plugin.so` 会在自己起来之后才 `set /use_sim_time true`
（同时发布 `/clock`）。所以：

- gzserver 启动时若 `/use_sim_time` 还不存在 → 它自己锁在**墙钟**上，
  并且**不会订阅 `/clock`**
- 之后启动的 mavros / laserMapping 读到 `true` → 用**仿真时间**

判断依据：`rosnode` 注册表里，用仿真时间的节点都会 `+SUB [/clock]`，
而问题会话中 `/gazebo` 只有 `+PUB [/clock]`，从不订阅。

### 正确顺序

```bash
roscore &
rosparam set use_sim_time true      # 必须在 gzserver 启动之前
make px4_sitl gazebo-classic_iris_mid360
```

若已经先起了 Gazebo：**必须 kill 掉 gzserver 重新启动**，改参数对已运行进程无效。

```bash
bash scripts/kill_sim.sh
rosparam set use_sim_time true
bash scripts/run_sim.sh px4
```

### 自检

```bash
rosparam get /use_sim_time
rostopic echo -n1 /livox/lidar/header/stamp
rostopic echo -n1 /mavros/imu/data/header/stamp   # 与上面同一数量级才算对
rosnode list | grep laserMapping
```

---

## 2. MAVROS 不发 `base_link` 的 TF

本项目的 `odom -> base_link` **只由 FAST-LIO 发布**。MAVROS 侧被显式关掉了：

- `mavros/launch/node.launch` 里 `<param name="local_position/tf/send" value="false"/>`
- `mavros/launch/px4_config.yaml` 里 `local_position.tf.send: false`、
  `global_position.tf.send: false`，且其 frame 用的是 `map` 而非 `odom`

所以如果你的 TF 树里只有 `base_link -> livox_link`，
说明 FAST-LIO 没在发 TF，而不是 mavros 配错了。

---

## 3. 点云的 `frame_id` 是 `Mid360::livox_link`

Gazebo 会自动给 link 名加上模型前缀，插件 SDF 里写的是 `livox_link`，
但实际发出的点云 `header.frame_id` 是 `Mid360::livox_link`。

RViz 里出现 "No transform from [Mid360::livox_link]" 时：

```bash
rosrun tf static_transform_publisher 0.07 0 0.072 0 0.3925 0 base_link "Mid360::livox_link" 100
```

FAST-LIO 的 `sim_handler` 只用 xyz + intensity，不看 frame，所以不影响建图。

---

## 4. `mid360.yaml` 与 `my_mid360.yaml` 必须选对

| 文件 | `lidar_type` | `imu_topic` | 用途 |
|---|---|---|---|
| `config/my_mid360.yaml` | `4`（MARSIM / `sensor_msgs/PointCloud2`） | `/mavros/imu/data` | **仿真** |
| `config/mid360.yaml` | `1`（AVIA / `livox_ros_driver::CustomMsg`） | `/livox/imu` | 真机 |

`mapping_mid360.launch` 加载的是 `my_mid360.yaml`。
仿真里 Gazebo 插件发的是 `sensor_msgs/PointCloud2`，也**不发布** `/livox/imu`
（仿真 IMU 来自 mavros），所以用真机那份配置会一个点都收不到。

另外注意：`time_sync_en` 的自同步逻辑只写在 `livox_pcl_cbk()`（AVIA 路径）里，
`standard_pcl_cbk()`（`lidar_type != 1` 走的路径）里没有，所以仿真下该参数不起作用。

---

## 5. 其它

- **conda**：装了 anaconda 时每个新终端先 `conda deactivate`，否则 ROS 的 python 会被顶掉。
- **残留进程**：`make px4_sitl` 异常退出后常留下 gzserver，先 `bash scripts/kill_sim.sh`。
- **换世界**：`make px4_sitl gazebo-classic_iris_mid360__warehouse`，
  注意模型名和世界名之间是**双下划线**（CMake 目标命名规则）。
