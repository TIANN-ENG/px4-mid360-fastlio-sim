// offboard_bt —— 用行为树（BehaviorTree.CPP v3）驱动的 PX4 Offboard 控制节点
//
// 和 offboard_takeoff / offboard_position_control / offboard_orbit 的区别：
//   那几个节点把"任务流程"写死在 C++ 的 switch-case 状态机里，改一次流程要重编译；
//   这个节点只提供一个【飞行原语库】，任务流程写成 XML 行为树，改流程只改 XML。
//
// 线程模型（为什么不是一个 while 循环搞定）：
//   +---------------- 主线程 ----------------+     +------ 推流线程 ------+
//   | ros::spinOnce()  更新 DroneState       |     | 固定 50Hz 读 setpoint |
//   | tree.tickRoot()  跑行为树 -> 更新 setpoint|-->| publish 到 /mavros/    |
//   +----------------------------------------+     +----------------------+
//   行为树的 tick 里可能有阻塞的服务调用（arm / set_mode），不能让它拖累发布频率，
//   否则 PX4 会因为 setpoint 断流触发 failsafe。详见 include/px4_control/drone_state.h。
//
// 运行（需先起好 仿真 + mavros）：
//   rosrun px4_control offboard_bt
//   rosrun px4_control offboard_bt _tree_file:=/abs/path/my_tree.xml
//   roslaunch px4_control offboard_bt.launch
//
// 参数（私有命名空间 ~）：
//   tree_file        行为树 XML 路径           默认 <pkg>/behavior_trees/orbit_mission.xml
//   tick_rate        行为树 tick 频率 Hz       默认 20
//   stream_rate      setpoint 发布频率 Hz      默认 50
//   prestream_time   切 OFFBOARD 前的预推流秒数 默认 2
//   watchdog         飞行中丢 OFFBOARD 时是否自动抢回  默认 true
//   exit_on_complete 任务结束后退出进程         默认 true
//   log_transitions  打印每次节点状态跳变       默认 false
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <px4_control/bt_nodes.h>
#include <px4_control/drone_state.h>

#include <memory>
#include <string>

namespace px4_bt {
namespace {

const char* kDefaultTreeRelPath = "behavior_trees/orbit_mission.xml";

/// 在成员初始化列表里读参数（此时 pnh_ 已经构造好了）
double readStreamRate(ros::NodeHandle& pnh)
{
  double rate = 50.0;
  pnh.param("stream_rate", rate, 50.0);
  return rate;
}

}  // namespace

class OffboardBt
{
public:
  OffboardBt()
    : nh_()
    , pnh_("~")
    , streamer_(nh_, "/mavros/setpoint_position/local", readStreamRate(pnh_))
  {
    pnh_.param("tick_rate", tick_rate_, 20.0);
    pnh_.param("prestream_time", prestream_time_, 2.0);
    pnh_.param("watchdog", watchdog_enabled_, true);
    pnh_.param("exit_on_complete", exit_on_complete_, true);
    pnh_.param("log_transitions", log_transitions_, false);

    std::string tree_file;
    pnh_.param<std::string>("tree_file", tree_file, std::string());
    tree_file_ = tree_file.empty() ? defaultTreePath() : tree_file;

    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardBt::stateCallback, this);
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
                              &OffboardBt::poseCallback, this);
    // 视觉目标（下视相机 ArUco），供 IsTargetVisible 之类的条件节点使用
    target_sub_ = nh_.subscribe("/aruco_down_cam/pose", 10,
                                &OffboardBt::targetCallback, this);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ctx_.nh       = &nh_;
    ctx_.state    = &state_;
    ctx_.streamer = &streamer_;
    ctx_.arming   = &arming_client_;
    ctx_.mode     = &mode_client_;

    ROS_INFO("offboard_bt: tree=%s tick=%.0fHz stream=%.0fHz prestream=%.1fs watchdog=%s",
             tree_file_.c_str(), tick_rate_, streamer_.rateHz(), prestream_time_,
             watchdog_enabled_ ? "on" : "off");
  }

  void run()
  {
    ros::Rate rate(tick_rate_);

    if (!waitForFcu(rate)) return;

    // 记录起飞点：所有相对坐标、环绕圆心都以它为原点
    ctx_.origin_x = state_.pose.pose.position.x;
    ctx_.origin_y = state_.pose.pose.position.y;
    ctx_.origin_z = state_.pose.pose.position.z;
    ROS_INFO("takeoff origin = (%.2f, %.2f, %.2f)", ctx_.origin_x, ctx_.origin_y,
             ctx_.origin_z);

    prestream(rate);

    BT::BehaviorTreeFactory factory;
    registerNodes(factory);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", &ctx_);  // 节点靠它拿 ROS 句柄，必须先于建树

    try {
      tree_ = factory.createTreeFromFile(tree_file_, blackboard);
    } catch (const std::exception& e) {
      ROS_FATAL("failed to load behavior tree [%s]: %s", tree_file_.c_str(), e.what());
      ros::shutdown();
      return;
    }
    ROS_INFO("behavior tree loaded, mission start");

    std::unique_ptr<BT::StdCoutLogger> logger;
    if (log_transitions_) logger.reset(new BT::StdCoutLogger(tree_));

    while (ros::ok()) {
      ros::spinOnce();

      // 安全层放在行为树【外面】：
      // 任务逻辑在树里，而"模式被 RC/超时踢掉"这种异常需要立刻抢回，
      // 又不能打断树的执行进度，所以单独做一个看门狗。
      if (watchdog_enabled_) watchdog();

      const BT::NodeStatus status = tree_.tickRoot();

      if (status == BT::NodeStatus::SUCCESS) {
        ROS_INFO("mission complete (SUCCESS)");
        finish(rate);
        return;
      }
      if (status == BT::NodeStatus::FAILURE) {
        ROS_ERROR("mission FAILED (FAILURE): see the last [BT] line above to find the failing node");
        finish(rate);
        return;
      }

      rate.sleep();
    }
  }

private:
  // ---------------- 启动阶段的三个小步骤 ----------------

  bool waitForFcu(ros::Rate& rate)
  {
    ROS_INFO("waiting for FCU and local pose ...");
    while (ros::ok()) {
      ros::spinOnce();
      if (state_.fcu.connected && state_.pose_valid) {
        ROS_INFO("FCU connected");
        return true;
      }
      rate.sleep();
    }
    return false;
  }

  /// PX4 规定：切 OFFBOARD 之前必须已经在收 setpoint。
  /// 这里预推流的是【当前位姿】（原地保持），比直接推目标高度安全 ——
  /// 万一节点是在空中重启的，也不会一进 OFFBOARD 就俯冲。
  void prestream(ros::Rate& rate)
  {
    streamer_.set(ctx_.origin_x, ctx_.origin_y, ctx_.origin_z,
                  yawFromQuaternion(state_.pose.pose.orientation));
    ROS_INFO("pre-streaming setpoints for %.1fs (hold current position) ...", prestream_time_);

    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < prestream_time_) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  void finish(ros::Rate& rate)
  {
    if (exit_on_complete_) {
      ros::shutdown();
      return;
    }
    // 不退出：继续维持最后一条 setpoint，方便现场观察
    ROS_WARN("exit_on_complete=false: holding position, no more ticks (Ctrl-C to quit)");
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  // ---------------- 安全看门狗 ----------------

  void watchdog()
  {
    if (!ctx_.offboard_recover) return;
    if (!state_.fcu.connected || !state_.fcu.armed) return;
    if (state_.fcu.mode == "OFFBOARD") return;

    ROS_WARN_THROTTLE(2.0, "OFFBOARD lost in flight (mode=%s), re-requesting ...",
                      state_.fcu.mode.c_str());
    ctx_.requestOffboard();
  }

  // ---------------- 回调 ----------------

  void stateCallback(const mavros_msgs::State::ConstPtr& msg)
  {
    state_.fcu = *msg;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    state_.pose       = *msg;
    state_.pose_valid = true;
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    state_.target       = *msg;
    state_.target_stamp = ros::Time::now();
  }

  // ---------------- 杂项 ----------------

  std::string defaultTreePath() const
  {
    const std::string pkg = ros::package::getPath("px4_control");
    if (!pkg.empty()) return pkg + "/" + kDefaultTreeRelPath;
    return kDefaultTreeRelPath;
  }

  // ---------------- 成员（注意初始化顺序要和构造函数一致）----------------

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  SetpointStreamer streamer_;

  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber target_sub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient mode_client_;

  DroneState   state_;
  BtContext    ctx_;
  BT::Tree     tree_;

  std::string tree_file_;
  double tick_rate_{20.0};
  double prestream_time_{2.0};
  bool   watchdog_enabled_{true};
  bool   exit_on_complete_{true};
  bool   log_transitions_{false};
};

}  // namespace px4_bt

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offboard_bt");
  px4_bt::OffboardBt node;
  node.run();
  return 0;
}
