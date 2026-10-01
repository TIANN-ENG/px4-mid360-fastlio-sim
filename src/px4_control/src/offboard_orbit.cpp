#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <algorithm>
#include <cmath>

/*
 * offboard_orbit —— 起飞 → 悬停 → 环绕 → 自动降落
 *
 * 状态机：
 *   WAIT_FCU -> ARM_OFFBOARD -> TAKEOFF -> HOVER -> ORBIT -> LAND -> DONE
 *
 * 坐标系：全部使用 MAVROS 的本地 ENU（x 东、y 北、z 上，单位 m，yaw 逆时针为正）。
 *        mavros 内部会自动转换成 PX4 的 NED，不需要自己换算。
 *
 * 运行（需先起好 仿真 + mavros）：
 *   rosrun px4_control offboard_orbit
 *   rosrun px4_control offboard_orbit _radius:=4.0 _height:=2.5 _speed:=1.2 _laps:=2 _clockwise:=true
 *
 * 参数（私有命名空间 ~）：
 *   height        目标高度 m                          默认 2.0
 *   radius        环绕半径 m                          默认 3.0
 *   speed         圆周线速度 m/s                      默认 1.0
 *   laps          环绕圈数                            默认 1.0
 *   hover_time    起飞后悬停秒数                      默认 5.0
 *   clockwise     顺时针环绕                          默认 false
 *   face_tangent  true=机头朝切线(前进方向)，false=机头固定朝东   默认 true
 *   takeoff_tol   起飞到位判定容差 m                  默认 0.25
 */

namespace {

// 只绕 z 轴旋转的四元数，避免为此引入 tf/tf2 依赖
geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

geometry_msgs::PoseStamped makeSetpoint(double x, double y, double z, double yaw)
{
    geometry_msgs::PoseStamped sp;
    sp.header.stamp = ros::Time::now();
    sp.header.frame_id = "map";
    sp.pose.position.x = x;
    sp.pose.position.y = y;
    sp.pose.position.z = z;
    sp.pose.orientation = yawToQuaternion(yaw);   // 必须给合法四元数，否则 yaw 会算出 NaN
    return sp;
}

}  // namespace

class OrbitController
{
public:
    OrbitController()
        : flight_state_(WAIT_FCU)
        , land_requested_(false)
    {
        ros::NodeHandle pnh("~");
        pnh.param("height",       height_,       2.0);
        pnh.param("radius",       radius_,       3.0);
        pnh.param("speed",        speed_,        1.0);
        pnh.param("laps",         laps_,         1.0);
        pnh.param("hover_time",   hover_time_,   5.0);
        pnh.param("clockwise",    clockwise_,    false);
        pnh.param("face_tangent", face_tangent_, true);
        pnh.param("takeoff_tol",  takeoff_tol_,  0.25);

        // 环绕角速度由线速度推出；半径过小时做保护，避免除零
        omega_ = speed_ / std::max(radius_, 0.1);

        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
        state_sub_ = nh_.subscribe(
            "/mavros/state", 10, &OrbitController::stateCallback, this);
        pose_sub_ = nh_.subscribe(
            "/mavros/local_position/pose", 10, &OrbitController::poseCallback, this);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("orbit node ready: height=%.1fm radius=%.1fm speed=%.1fm/s laps=%.1f %s",
                 height_, radius_, speed_, laps_,
                 clockwise_ ? "clockwise" : "counter-clockwise");
    }

    void run()
    {
        ros::Rate rate(30.0);   // >2Hz 是 PX4 对 offboard 的硬性要求

        // PX4 规定：切 OFFBOARD 之前必须已经在持续接收 setpoint
        ROS_INFO("pre-streaming setpoints for 2s before OFFBOARD ...");
        for (int i = 0; ros::ok() && i < 60; ++i) {
            setpoint_pub_.publish(makeSetpoint(0.0, 0.0, height_, 0.0));
            ros::spinOnce();
            rate.sleep();
        }

        while (ros::ok()) {
            // 飞行中若被 RC 接管或超时踢出 OFFBOARD，要重新申请，否则 setpoint 会被静默忽略
            if (flying() && fcu_state_.armed && fcu_state_.mode != "OFFBOARD") {
                ROS_WARN_THROTTLE(1.0, "OFFBOARD lost (mode=%s), re-requesting ...",
                                  fcu_state_.mode.c_str());
                requestOffboard();
            }

            switch (flight_state_) {
                case WAIT_FCU:     waitFcu();     break;
                case ARM_OFFBOARD: armOffboard(); break;
                case TAKEOFF:      takeoff();     break;
                case HOVER:        hover();       break;
                case ORBIT:        orbit();       break;
                case LAND:         if (land()) return; break;
                case DONE:         return;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    enum FlightState { WAIT_FCU = 0, ARM_OFFBOARD, TAKEOFF, HOVER, ORBIT, LAND, DONE };

    bool flying() const
    {
        return flight_state_ == TAKEOFF || flight_state_ == HOVER || flight_state_ == ORBIT;
    }

    // ---------------- 回调 ----------------
    void stateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        fcu_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = *msg;
    }

    // ---------------- 工具 ----------------
    void publishSetpoint(double x, double y, double z, double yaw)
    {
        setpoint_pub_.publish(makeSetpoint(x, y, z, yaw));
    }

    bool requestOffboard()
    {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        return mode_client_.call(srv) && srv.response.mode_sent;
    }

    // ---------------- WAIT_FCU ----------------
    void waitFcu()
    {
        if (!fcu_state_.connected) {
            ROS_INFO_THROTTLE(2.0, "waiting for FCU (mavros) ...");
            return;
        }
        if (current_pose_.header.stamp.isZero()) {
            ROS_INFO_THROTTLE(2.0, "waiting for /mavros/local_position/pose ...");
            return;
        }

        // 记录起飞点，同时作为环绕圆心
        center_x_ = current_pose_.pose.position.x;
        center_y_ = current_pose_.pose.position.y;
        ROS_INFO("FCU connected. center = (%.2f, %.2f), ground z = %.2f",
                 center_x_, center_y_, current_pose_.pose.position.z);
        flight_state_ = ARM_OFFBOARD;
    }

    // ---------------- ARM_OFFBOARD ----------------
    void armOffboard()
    {
        // 分轮次：这一轮只切模式，下一轮再解锁（同一轮连做容易失败）
        if (fcu_state_.mode != "OFFBOARD") {
            if (requestOffboard()) ROS_INFO("OFFBOARD requested");
            return;
        }

        if (!fcu_state_.armed) {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (arming_client_.call(srv) && srv.response.success) ROS_INFO("armed");
            return;
        }

        ROS_INFO("OFFBOARD + armed -> takeoff");
        flight_state_ = TAKEOFF;
    }

    // ---------------- TAKEOFF ----------------
    void takeoff()
    {
        publishSetpoint(center_x_, center_y_, height_, 0.0);

        const double dz = std::fabs(current_pose_.pose.position.z - height_);
        if (dz < takeoff_tol_) {
            ROS_INFO("takeoff complete at z=%.2f", current_pose_.pose.position.z);
            phase_start_ = ros::Time::now();
            flight_state_ = HOVER;
        }
    }

    // ---------------- HOVER ----------------
    void hover()
    {
        publishSetpoint(center_x_, center_y_, height_, 0.0);

        if ((ros::Time::now() - phase_start_).toSec() > hover_time_) {
            orbit_start_ = ros::Time::now();
            ROS_INFO("hover done -> orbit (R=%.1fm, v=%.1fm/s, %.1f lap)",
                     radius_, speed_, laps_);
            flight_state_ = ORBIT;
        }
    }

    // ---------------- ORBIT ----------------
    void orbit()
    {
        const double t     = (ros::Time::now() - orbit_start_).toSec();
        const double dir   = clockwise_ ? -1.0 : 1.0;
        const double theta = dir * omega_ * t;

        const double x = center_x_ + radius_ * std::cos(theta);
        const double y = center_y_ + radius_ * std::sin(theta);

        // 机头朝圆周前进方向（切线）：theta + dir*90° 正好是速度方向
        const double yaw = face_tangent_ ? (theta + dir * M_PI / 2.0) : 0.0;
        publishSetpoint(x, y, height_, yaw);

        const double total_time = laps_ * 2.0 * M_PI / omega_;
        if (t >= total_time) {
            ROS_INFO("orbit complete, start landing");
            flight_state_ = LAND;
        }
    }

    // ---------------- LAND ----------------
    bool land()
    {
        if (!land_requested_) {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "AUTO.LAND";
            if (mode_client_.call(srv) && srv.response.mode_sent) {
                ROS_INFO("AUTO.LAND requested");
                land_requested_ = true;
                phase_start_ = ros::Time::now();
            }
            return false;   // 下一轮再判断是否落地
        }

        // AUTO.LAND 由 PX4 自己接管，这里不再发 setpoint
        if (!fcu_state_.armed) {
            ROS_INFO("disarmed -> land complete");
            return true;
        }

        const double z = current_pose_.pose.position.z;
        if (z < 0.15) {
            ROS_INFO("landed at z=%.2f", z);
            return true;
        }

        if ((ros::Time::now() - phase_start_).toSec() > 60.0) {
            ROS_WARN("landing timeout (z=%.2f), stop waiting", z);
            return true;
        }
        return false;
    }

    // ---------------- 成员 ----------------
    ros::NodeHandle nh_;
    ros::Publisher  setpoint_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient mode_client_;

    mavros_msgs::State         fcu_state_;      // mavros 上报的飞控状态
    geometry_msgs::PoseStamped current_pose_;   // 当前位姿（ENU）

    FlightState flight_state_;                  // 状态机
    bool land_requested_;

    double height_       = 2.0;
    double radius_       = 3.0;
    double speed_        = 1.0;
    double laps_         = 1.0;
    double hover_time_   = 5.0;
    double takeoff_tol_  = 0.25;
    bool   clockwise_    = false;
    bool   face_tangent_ = true;
    double omega_        = 0.0;

    double center_x_ = 0.0;
    double center_y_ = 0.0;
    ros::Time phase_start_;
    ros::Time orbit_start_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_orbit");
    OrbitController controller;
    controller.run();
    return 0;
}
