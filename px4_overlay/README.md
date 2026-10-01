# PX4 overlay

本目录只保存**对 PX4-Autopilot 的最小改动**，不 vendor 整个上游仓库（14G，其中 12G 是 build）。
`scripts/bootstrap.sh` 会 clone 官方 PX4 并自动应用这里的补丁。

## 改动清单（共 4 处）

| 文件 | 改动 | 作用 |
|---|---|---|
| `src/modules/simulation/simulator_mavlink/...` → 见补丁 | 在 `models` 列表加 `iris_mid360` | 生成 `make px4_sitl gazebo-classic_iris_mid360` 目标 |
| `ROMFS/.../init.d-posix/airframes/CMakeLists.txt` | 注册 `10020_gazebo-classic_iris_mid360` | 让 airframe 进入 ROMFS |
| `Tools/simulation/gazebo-classic/sitl_run.sh` | gzserver 加 `-s libgazebo_ros_api_plugin.so` | 提供 `/clock` 与 `/use_sim_time`，让 ROS 侧用仿真时间 |
| `ROMFS/.../airframes/10020_gazebo-classic_iris_mid360` | 新增文件 | iris_mid360 的机型定义（airframe id = 10020） |

对应补丁：`patches/px4-iris-mid360.patch`（`git apply -p1` 于 PX4 根目录）

## 手工应用

```bash
PX4_ROOT=$HOME/PX4-Autopilot

# 1. 打补丁（3 个已有文件的改动）
git -C "$PX4_ROOT" apply --check px4_overlay/patches/px4-iris-mid360.patch && \
git -C "$PX4_ROOT" apply         px4_overlay/patches/px4-iris-mid360.patch

# 2. 放入新增的 airframe 文件
cp px4_overlay/airframes/10020_gazebo-classic_iris_mid360 \
   "$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes/"

# 3. 重新编译
make -C "$PX4_ROOT" px4_sitl_default -j$(nproc)
```

若补丁因上游更新而冲突，`git apply --3way` 或对照上表手工改这 3 处即可。

## 为什么模型不放在 PX4 里

`Mid360` 与 `iris_mid360` 两个 Gazebo 模型由 `Mid360_px4_sim_plugin` 仓库提供，
并通过环境变量让 Gazebo 找到它们：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<repo>/src/Mid360_px4_sim_plugin/livox_laser_simulation/models
```

好处：

- 不需要往 PX4 的 `sitl_gazebo-classic` 子模块里拷文件（避免污染 submodule、重复维护两份）
- 插件与模型同源同版本，改模型只改一处
- 插件编译出的 `liblivox_laser_simulation.so` 通过 `GAZEBO_PLUGIN_PATH` 指向 `devel/lib` 即可
