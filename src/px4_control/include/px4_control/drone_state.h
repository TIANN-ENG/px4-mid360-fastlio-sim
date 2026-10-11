// 行为树控制层的共享数据与"指令流"。
//
// 这里只放三样东西：
//   1) Setpoint      —— 一条位置指令（MAVROS 本地 ENU）
//   2) SetpointStreamer —— 后台定频把最新指令发给 PX4 的线程
//   3) DroneState / BtContext —— 飞控状态、位姿、视觉目标，以及传给 BT 节点的上下文
//
// 线程模型（很重要，改代码前先看一眼）：
//   主线程   ：ros::spinOnce() 收回调 -> 更新 DroneState -> tree.tickRoot() 跑行为树
//   推流线程 ：只读 SetpointStreamer 内部那份 setpoint，定频 publish
// 也就是说 DroneState 是"主线程写、主线程读"，不需要加锁；
// 唯一跨线程的东西是 Setpoint，用 SetpointStreamer 内的 mutex 保护。
#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

namespace px4_bt {

// ---------------------------------------------------------------- 小工具

/// 只绕 z 轴旋转的四元数，省掉一个 tf 依赖
inline geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

/// 从四元数里抠出偏航角（同样是为了不引 tf）
inline double yawFromQuaternion(const geometry_msgs::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ---------------------------------------------------------------- 指令

/// 一条位置指令（MAVROS 本地 ENU：x 东 / y 北 / z 上，单位 m，yaw 逆时针为正）
struct Setpoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool   valid{false};
};

/**
 * 后台定频发布 /mavros/setpoint_position/local。
 *
 * 为什么要单独开线程？
 *   PX4 要求 OFFBOARD 期间 setpoint 频率 > 2Hz，断流会触发 failsafe。而行为树的
 *   tick 里会调用 arm / set_mode 这类【阻塞服务调用】，如果"发指令"和"想逻辑"
 *   挤在同一个循环里，服务一慢就会把发布周期一起拖垮。
 *   所以：推流线程只管发（固定 50Hz），行为树只管想。
 *
 * 由于流线程一直在发最后一条指令，行为树节点不需要每 tick 都重新 publish，
 * 只要在状态变化时 set() 一次即可。
 */
class SetpointStreamer
{
public:
  SetpointStreamer(ros::NodeHandle& nh, const std::string& topic, double rate_hz)
    : rate_hz_(rate_hz)
  {
    pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic, 10);
    thread_ = std::thread(&SetpointStreamer::loop, this);
  }

  ~SetpointStreamer()
  {
    running_ = false;
    if (thread_.joinable()) thread_.join();
  }

  SetpointStreamer(const SetpointStreamer&) = delete;
  SetpointStreamer& operator=(const SetpointStreamer&) = delete;

  /// 更新指令并指定偏航角（BT 节点调用，线程安全）
  void set(double x, double y, double z, double yaw)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    sp_.x = x;
    sp_.y = y;
    sp_.z = z;
    sp_.yaw = yaw;
    sp_.valid = true;
  }

  /// 只改位置，偏航角沿用上一条
  void set(double x, double y, double z)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    sp_.x = x;
    sp_.y = y;
    sp_.z = z;
    sp_.valid = true;
  }

  /// 只改偏航角
  void setYaw(double yaw)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    sp_.yaw = yaw;
    sp_.valid = true;
  }

  Setpoint get() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return sp_;
  }

  double rateHz() const { return rate_hz_; }

private:
  void loop()
  {
    ros::Rate rate(rate_hz_);
    while (ros::ok() && running_) {
      Setpoint sp;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        sp = sp_;
      }
      if (sp.valid) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.position.x = sp.x;
        msg.pose.position.y = sp.y;
        msg.pose.position.z = sp.z;
        // 必须给合法四元数，否则 yaw 会算出 NaN
        msg.pose.orientation = yawToQuaternion(sp.yaw);
        pub_.publish(msg);
      }
      rate.sleep();
    }
  }

  ros::Publisher pub_;
  mutable std::mutex mtx_;
  Setpoint sp_;
  double rate_hz_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

// ---------------------------------------------------------------- 状态

/// 飞控 + 位姿 + 视觉目标的共享快照（回调写、BT 读，都在主线程，无需加锁）
struct DroneState
{
  mavros_msgs::State         fcu;          // /mavros/state
  geometry_msgs::PoseStamped pose;         // /mavros/local_position/pose
  bool                       pose_valid{false};

  geometry_msgs::PoseStamped target;       // /aruco_down_cam/pose
  ros::Time                  target_stamp; // 最近一次收到目标的时刻

  /// 目标位姿是否"够新鲜"（视觉节点掉线时条件节点要能判出来）
  bool targetFresh(double timeout_s) const
  {
    return !target_stamp.isZero() &&
           (ros::Time::now() - target_stamp).toSec() <= timeout_s;
  }
};

/// 传给 BT 节点的上下文：挂在黑板的 "ctx" 键上，避免用全局变量
struct BtContext
{
  ros::NodeHandle*    nh{nullptr};
  DroneState*         state{nullptr};
  SetpointStreamer*   streamer{nullptr};
  ros::ServiceClient* arming{nullptr};
  ros::ServiceClient* mode{nullptr};

  /// 起飞点（记录的是 BT 开始工作那一刻的位姿），所有相对坐标都以它为原点
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};

  /// 是否允许看门狗在"已解锁但不在 OFFBOARD"时把模式抢回来。
  /// 进 AUTO.LAND 或人工接管时必须置 false，否则看门狗会和降落流程打架。
  bool offboard_recover{true};

  bool requestOffboard()
  {
    if (!mode) return false;
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = "OFFBOARD";
    return mode->call(srv) && srv.response.mode_sent;
  }

  bool setArmed(bool value)
  {
    if (!arming) return false;
    mavros_msgs::CommandBool srv;
    srv.request.value = value;
    return arming->call(srv) && srv.response.success;
  }
};

}  // namespace px4_bt
