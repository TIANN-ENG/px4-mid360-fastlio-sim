// 行为树节点库的实现。
//
// 节点分两类：
//   条件节点（ConditionNode）—— 只读，必须"瞬时"返回，绝不能阻塞
//   动作节点（StatefulActionNode / SyncActionNode）
//     - Stateful：会跨多个 tick，用 onStart/onRunning/onHalted 维护自己的状态，
//                 适合"做到某件事为止"（起飞到位、飞到航点、绕完 N 圈、降落）
//     - Sync    ：一个 tick 内办完，适合 arm / disarm 这种一次性的服务调用
//
// 想加自己的节点：照抄下面的写法，然后在文件末尾 registerNodes() 里注册一行。
#include <px4_control/bt_nodes.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace px4_bt {
namespace {

/// 从黑板上取上下文；取不到就直接抛，免得后面空指针崩溃
BtContext* ctxOf(const BT::NodeConfiguration& config)
{
  BtContext* ctx = nullptr;
  if (config.blackboard && config.blackboard->get("ctx", ctx) && ctx != nullptr) {
    return ctx;
  }
  throw std::runtime_error(
      "BT node has no context: set blackboard key \"ctx\" before createTreeFromFile()");
}

// ================================================================ 条件节点

/// 飞控（MAVROS）是否在线
class IsFcuConnected : public BT::ConditionNode
{
public:
  IsFcuConnected(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return ctx_->state->fcu.connected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

/// 是否已解锁
class IsArmed : public BT::ConditionNode
{
public:
  IsArmed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return ctx_->state->fcu.armed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

/// 飞控是否处于 OFFBOARD
class IsOffboard : public BT::ConditionNode
{
public:
  IsOffboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return ctx_->state->fcu.mode == "OFFBOARD" ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

/// 当前位姿是否已经到位（对比的是"当前 setpoint"）
class IsAtSetpoint : public BT::ConditionNode
{
public:
  IsAtSetpoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("tol", 0.3, "当前位姿与当前 setpoint 的最大允许误差 m")};
  }

  BT::NodeStatus tick() override
  {
    double tol = 0.3;
    getInput("tol", tol);

    const Setpoint sp = ctx_->streamer->get();
    if (!sp.valid || !ctx_->state->pose_valid) return BT::NodeStatus::FAILURE;

    const auto& p = ctx_->state->pose.pose.position;
    const double dx = p.x - sp.x;
    const double dy = p.y - sp.y;
    const double dz = p.z - sp.z;
    const double d  = std::sqrt(dx * dx + dy * dy + dz * dz);

    return (d <= tol) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

/// 视觉目标是否可见（/aruco_down_cam/pose 是否还有新鲜数据）
class IsTargetVisible : public BT::ConditionNode
{
public:
  IsTargetVisible(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout", 0.5, "目标位姿的新鲜度阈值 s")};
  }

  BT::NodeStatus tick() override
  {
    double timeout = 0.5;
    getInput("timeout", timeout);
    return ctx_->state->targetFresh(timeout) ? BT::NodeStatus::SUCCESS
                                             : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

// ================================================================ 动作节点

/// 一定要进入 OFFBOARD 模式（切不过去就一直 RUNNING，超时才 FAILURE）
class EnsureOffboard : public BT::StatefulActionNode
{
public:
  EnsureOffboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout", 10.0, "等待 OFFBOARD 生效的秒数")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("timeout", timeout_);
    start_    = ros::Time::now();
    last_req_ = ros::Time(0);  // 立刻发一次
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (ctx_->state->fcu.mode == "OFFBOARD") {
      ROS_INFO("[BT] OFFBOARD active");
      return BT::NodeStatus::SUCCESS;
    }

    if (ctx_->state->fcu.connected &&
        (ros::Time::now() - last_req_).toSec() > 1.0) {
      ctx_->requestOffboard();
      last_req_ = ros::Time::now();
      ROS_INFO("[BT] requesting OFFBOARD ...");
    }

    if (elapsed() > timeout_) {
      ROS_ERROR("[BT] timeout waiting for OFFBOARD (mode=%s)",
                ctx_->state->fcu.mode.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  double elapsed() const { return (ros::Time::now() - start_).toSec(); }

  BtContext* ctx_;
  double     timeout_{10.0};
  ros::Time  start_;
  ros::Time  last_req_;
};

/// 一定要解锁（解锁命令会重试，超时才 FAILURE）
class EnsureArmed : public BT::StatefulActionNode
{
public:
  EnsureArmed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout", 10.0, "等待解锁成功的秒数")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("timeout", timeout_);
    start_    = ros::Time::now();
    last_req_ = ros::Time(0);
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (ctx_->state->fcu.armed) {
      ROS_INFO("[BT] armed");
      return BT::NodeStatus::SUCCESS;
    }

    if (ctx_->state->fcu.connected &&
        (ros::Time::now() - last_req_).toSec() > 1.0) {
      ctx_->setArmed(true);
      last_req_ = ros::Time::now();
      ROS_INFO("[BT] requesting arming ...");
    }

    if (elapsed() > timeout_) {
      ROS_ERROR("[BT] arming timeout");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  double elapsed() const { return (ros::Time::now() - start_).toSec(); }

  BtContext* ctx_;
  double     timeout_{10.0};
  ros::Time  start_;
  ros::Time  last_req_;
};

/// 解锁（一次性，成功/失败立刻返回）
class Arm : public BT::SyncActionNode
{
public:
  Arm(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (ctx_->state->fcu.armed) return BT::NodeStatus::SUCCESS;
    return ctx_->setArmed(true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

/// 上锁（一次性，通常放在"任务结束/异常分支"里）
class Disarm : public BT::SyncActionNode
{
public:
  Disarm(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!ctx_->state->fcu.armed) return BT::NodeStatus::SUCCESS;
    return ctx_->setArmed(false) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  BtContext* ctx_;
};

/// 起飞到指定高度（相对起飞点地面）
class Takeoff : public BT::StatefulActionNode
{
public:
  Takeoff(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("height", 2.0, "相对起飞点地面的目标高度 m"),
        BT::InputPort<double>("tol", 0.25, "到高判定容差 m"),
        BT::InputPort<double>("timeout", 30.0, "超时 s"),
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("height", height_);
    getInput("tol", tol_);
    getInput("timeout", timeout_);
    start_ = ros::Time::now();
    // 保持起飞瞬间的机头朝向，否则一起飞就会自己转向 yaw=0（正东）
    if (ctx_->state->pose_valid) {
      yaw_ = yawFromQuaternion(ctx_->state->pose.pose.orientation);
    }
    ROS_INFO("[BT] Takeoff -> z = %.2f m", ctx_->origin_z + height_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double target_z = ctx_->origin_z + height_;
    // 持续锁住目标，防止推流缓存里还留着上一条别的指令
    ctx_->streamer->set(ctx_->origin_x, ctx_->origin_y, target_z, yaw_);

    const double z = ctx_->state->pose.pose.position.z;
    if (std::fabs(z - target_z) < tol_) {
      ROS_INFO("[BT] Takeoff done (z = %.2f)", z);
      return BT::NodeStatus::SUCCESS;
    }
    if ((ros::Time::now() - start_).toSec() > timeout_) {
      ROS_ERROR("[BT] Takeoff timeout: z = %.2f target %.2f", z, target_z);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  BtContext* ctx_;
  double     height_{2.0};
  double     tol_{0.25};
  double     timeout_{30.0};
  double     yaw_{0.0};
  ros::Time  start_;
};

/// 原地悬停若干秒（保持当前 setpoint，推流线程会一直发）
class Hover : public BT::StatefulActionNode
{
public:
  Hover(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration", 3.0, "悬停秒数")};
  }

  BT::NodeStatus onStart() override
  {
    getInput("duration", duration_);
    start_ = ros::Time::now();
    ROS_INFO("[BT] Hover %.1f s", duration_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((ros::Time::now() - start_).toSec() >= duration_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  BtContext* ctx_;
  double     duration_{3.0};
  ros::Time  start_;
};

/// 相对起飞点平移到某个位置（前进/后退/左右/升降都靠它）
///
/// x/y/z 都是相对【起飞点】的位移，z 相对起飞点的地面。
/// 例如 x=2, z=1 表示飞到起点东侧 2m、离地 1m 的位置。
class GotoRelative : public BT::StatefulActionNode
{
public:
  GotoRelative(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("x", 0.0, "相对起飞点的东向位移 m"),
        BT::InputPort<double>("y", 0.0, "相对起飞点的北向位移 m"),
        BT::InputPort<double>("z", 0.0, "相对起飞点地面的高度 m"),
        // 不给默认值 -> 端口可选：没写就自动让机头朝运动方向
        BT::InputPort<double>("yaw", "绝对偏航角 rad；不写则机头朝运动方向"),
        BT::InputPort<double>("tol", 0.3, "到位容差 m"),
        BT::InputPort<double>("timeout", 20.0, "超时 s"),
    };
  }

  BT::NodeStatus onStart() override
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    getInput("x", x);
    getInput("y", y);
    getInput("z", z);
    getInput("tol", tol_);
    getInput("timeout", timeout_);

    target_x_ = ctx_->origin_x + x;
    target_y_ = ctx_->origin_y + y;
    target_z_ = ctx_->origin_z + z;

    // 端口没写默认值 -> 这里返回失败，说明用户没指定 yaw，那就自动朝运动方向
    has_yaw_ = static_cast<bool>(getInput("yaw", target_yaw_));
    start_   = ros::Time::now();

    ROS_INFO("[BT] GotoRelative -> (%.2f, %.2f, %.2f)%s", target_x_, target_y_, target_z_,
             has_yaw_ ? " yaw specified" : "");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto& p = ctx_->state->pose.pose.position;

    // 默认保持当前机头朝向；只有"还在走"的时候才转向运动方向
    double yaw = ctx_->state->pose_valid
                     ? yawFromQuaternion(ctx_->state->pose.pose.orientation)
                     : 0.0;
    if (has_yaw_) {
      yaw = target_yaw_;
    } else {
      const double dx = target_x_ - p.x;
      const double dy = target_y_ - p.y;
      if (std::sqrt(dx * dx + dy * dy) > 0.2) yaw = std::atan2(dy, dx);
    }
    ctx_->streamer->set(target_x_, target_y_, target_z_, yaw);

    const double dx = p.x - target_x_;
    const double dy = p.y - target_y_;
    const double dz = p.z - target_z_;
    const double d  = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (d < tol_) {
      ROS_INFO("[BT] target reached (d = %.2f)", d);
      return BT::NodeStatus::SUCCESS;
    }
    if ((ros::Time::now() - start_).toSec() > timeout_) {
      ROS_WARN("[BT] waypoint timeout (d = %.2f), failing", d);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  BtContext* ctx_;
  double     target_x_{0.0};
  double     target_y_{0.0};
  double     target_z_{0.0};
  double     target_yaw_{0.0};
  bool       has_yaw_{false};
  double     tol_{0.3};
  double     timeout_{20.0};
  ros::Time  start_;
};

/// 以起飞点为圆心绕圈（与 offboard_orbit 的 ORBIT 段等价）
class Orbit : public BT::StatefulActionNode
{
public:
  Orbit(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("radius", 3.0, "半径 m"),
        BT::InputPort<double>("speed", 1.0, "圆周线速度 m/s"),
        BT::InputPort<double>("laps", 1.0, "圈数，<=0 表示不绕直接成功"),
        BT::InputPort<double>("clockwise", false, "true = 顺时针"),
        BT::InputPort<double>("face_tangent", true, "true = 机头朝切线方向"),
        BT::InputPort<double>("z", "相对起飞点地面的高度 m；不写则保持当前高度"),
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("radius", radius_);
    getInput("speed", speed_);
    getInput("laps", laps_);
    getInput("clockwise", clockwise_);
    getInput("face_tangent", face_tangent_);

    double z = 0.0;
    if (getInput("z", z)) {
      target_z_ = ctx_->origin_z + z;
    } else {
      target_z_ = ctx_->state->pose.pose.position.z;  // 保持当前高度
    }

    omega_ = speed_ / std::max(radius_, 0.1);  // 半径过小时保护一下，避免除零
    // face_tangent=false 时"保持机头不动"，锁住进入环绕那一刻的朝向
    yaw_latched_ = ctx_->state->pose_valid
                       ? yawFromQuaternion(ctx_->state->pose.pose.orientation)
                       : 0.0;
    start_ = ros::Time::now();

    ROS_INFO("[BT] Orbit R=%.1fm v=%.1fm/s laps=%.1f %s @ z=%.2f", radius_, speed_, laps_,
             clockwise_ ? "CW" : "CCW", target_z_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double t     = (ros::Time::now() - start_).toSec();
    const double dir   = clockwise_ ? -1.0 : 1.0;
    const double theta = dir * omega_ * t;

    const double x = ctx_->origin_x + radius_ * std::cos(theta);
    const double y = ctx_->origin_y + radius_ * std::sin(theta);
    // 机头朝圆周前进方向（切线）：theta + dir*90° 正好是速度方向
    const double yaw = face_tangent_ ? (theta + dir * M_PI / 2.0) : yaw_latched_;

    ctx_->streamer->set(x, y, target_z_, yaw);

    const double total = std::fabs(laps_) * 2.0 * M_PI / omega_;
    if (t >= total) {
      ROS_INFO("[BT] Orbit done");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  BtContext* ctx_;
  double     radius_{3.0};
  double     speed_{1.0};
  double     laps_{1.0};
  double     omega_{1.0};
  bool       clockwise_{false};
  bool       face_tangent_{true};
  double     target_z_{0.0};
  double     yaw_latched_{0.0};
  ros::Time  start_;
};

/// 请求 AUTO.LAND 并等落地（交给 PX4 自己降落）
class Land : public BT::StatefulActionNode
{
public:
  Land(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), ctx_(ctxOf(config)) {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("z_threshold", 0.15, "低于这个高度算落地 m"),
        BT::InputPort<double>("timeout", 60.0, "降落超时 s"),
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("z_threshold", z_threshold_);
    getInput("timeout", timeout_);

    // 关键：告诉看门狗别再抢 OFFBOARD 了，否则会和 AUTO.LAND 打架
    ctx_->offboard_recover = false;

    if (ctx_->mode) {
      mavros_msgs::SetMode srv;
      srv.request.custom_mode = "AUTO.LAND";
      if (ctx_->mode->call(srv) && srv.response.mode_sent) {
        ROS_INFO("[BT] AUTO.LAND requested");
      } else {
        ROS_WARN("[BT] AUTO.LAND request failed, watching altitude");
      }
    }
    start_ = ros::Time::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // AUTO.LAND 由 PX4 自己接管，这里不再发 setpoint
    if (!ctx_->state->fcu.armed) {
      ROS_INFO("[BT] disarmed, landing complete");
      return BT::NodeStatus::SUCCESS;
    }
    const double z = ctx_->state->pose.pose.position.z;
    if (z < z_threshold_) {
      ROS_INFO("[BT] landed (z = %.2f)", z);
      return BT::NodeStatus::SUCCESS;
    }
    if ((ros::Time::now() - start_).toSec() > timeout_) {
      ROS_ERROR("[BT] landing timeout (z = %.2f)", z);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  BtContext* ctx_;
  double     z_threshold_{0.15};
  double     timeout_{60.0};
  ros::Time  start_;
};

}  // namespace

void registerNodes(BT::BehaviorTreeFactory& factory)
{
  // 条件
  factory.registerNodeType<IsFcuConnected>("IsFcuConnected");
  factory.registerNodeType<IsArmed>("IsArmed");
  factory.registerNodeType<IsOffboard>("IsOffboard");
  factory.registerNodeType<IsAtSetpoint>("IsAtSetpoint");
  factory.registerNodeType<IsTargetVisible>("IsTargetVisible");

  // 动作
  factory.registerNodeType<EnsureOffboard>("EnsureOffboard");
  factory.registerNodeType<EnsureArmed>("EnsureArmed");
  factory.registerNodeType<Arm>("Arm");
  factory.registerNodeType<Disarm>("Disarm");
  factory.registerNodeType<Takeoff>("Takeoff");
  factory.registerNodeType<Hover>("Hover");
  factory.registerNodeType<GotoRelative>("GotoRelative");
  factory.registerNodeType<Orbit>("Orbit");
  factory.registerNodeType<Land>("Land");
}

}  // namespace px4_bt
