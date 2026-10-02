#include <ros/ros.h>

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

/*
 * offboard_orbit —— 起飞 / 悬停 / 相对平移(前进后退等) / 环绕 / 自动降落
 *
 * 状态机：
 *   WAIT_FCU -> ARM_OFFBOARD -> TAKEOFF -> HOVER -> MOVE -> ORBIT -> LAND -> DONE
 *
 * 坐标系：全部使用 MAVROS 的本地 ENU（x 东、y 北、z 上，单位 m，yaw 逆时针为正）。
 *        mavros 内部会自动转换成 PX4 的 NED，不需要自己换算。
 *
 * 运行（需先起好 仿真 + mavros）：
 *   rosrun px4_control offboard_orbit
 *   rosrun px4_control offboard_orbit _height:=2.5 _radius:=4.0 _speed:=1.2 _laps:=2
 *
 * 参数（私有命名空间 ~）：
 *   height          目标高度 m                          默认 2.0
 *   hover_time      起飞后悬停秒数                      默认 5.0
 *   waypoints       相对起飞点的平移序列（见下）        默认 空（跳过 MOVE）
 *   waypoint_tol    到位判定容差 m                      默认 0.3
 *   waypoint_timeout 单个航点超时秒数                   默认 20.0
 *   radius          环绕半径 m                          默认 3.0
 *   speed           圆周线速度 m/s                      默认 1.0
 *   laps            环绕圈数；<=0 表示不环绕            默认 1.0
 *   clockwise       顺时针环绕                          默认 false
 *   face_tangent    true=机头朝切线(前进方向)           默认 true
 *   takeoff_tol     起飞到位判定容差 m                  默认 0.25
 *
 * waypoints 写法（每个元素 = [dx, dy, dz] 或 [dx, dy, dz, yaw]）：
 *   dx/dy 相对起飞点的水平位移（+x 东 = 前进，+y 北 = 左移）
 *   dz    相对【飞行高度】的升降（+上升，-下降），不是相对地面
 *   yaw   可选，绝对偏航角(rad)；不写则自动机头朝向运动方向
 *
 *   例（roslaunch 里）：
 *     <rosparam param="waypoints">[[2,0,0], [0,0,0.5], [-2,0,0], [0,0,0]]</rosparam>
 *        前进 2m -> 升高 0.5m -> 后退 2m（回到起飞点上方）-> 降回原高度
 *
 *   例（rosrun 单个航点）：
 *     rosrun px4_control offboard_orbit _waypoints:="[2,0,0]"
 *
 * 说明：
 *   - 环绕的圆心固定为【起飞点】，与 waypoints 无关，便于预期轨迹
 *   - 若 waypoints 为空则跳过 MOVE；若 laps<=0 则跳过 ORBIT；两者可独立开关
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

// XmlRpcValue -> double（int 和 double 两种类型都要处理，否则会断言失败）
double xmlToDouble(const XmlRpc::XmlRpcValue& v)
{
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<double>(static_cast<int>(v));
    }
    return static_cast<double>(v);
}

}  // namespace

struct Waypoint
{
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;          // 相对飞行高度
    double yaw = 0.0;
    bool   has_yaw = false;
};

class OrbitController
{
public:
    OrbitController()
        : flight_state_(WAIT_FCU)
        , land_requested_(false)
        , wp_index_(0)
    {
        ros::NodeHandle pnh("~");
        pnh.param("height",           height_,           2.0);
        pnh.param("hover_time",       hover_time_,       5.0);
        pnh.param("waypoint_tol",     waypoint_tol_,     0.3);
        pnh.param("waypoint_timeout", waypoint_timeout_, 20.0);
        pnh.param("radius",           radius_,           3.0);
        pnh.param("speed",            speed_,            1.0);
        pnh.param("laps",             laps_,             1.0);
        pnh.param("clockwise",        clockwise_,        false);
        pnh.param("face_tangent",     face_tangent_,     true);
        pnh.param("takeoff_tol",      takeoff_tol_,      0.25);

        parseWaypoints(pnh);

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

        ROS_INFO("orbit node ready: height=%.1fm hover=%.1fs waypoints=%zu orbit=%s%.1f lap R=%.1fm v=%.1fm/s",
                 height_, hover_time_, waypoints_.size(),
                 laps_ > 0.0 ? "" : "off ",
                 laps_ > 0.0 ? laps_ : 0.0, radius_, speed_);
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
                case MOVE:         move();        break;
                case ORBIT:        orbit();       break;
                case LAND:         if (land()) return; break;
                case DONE:         return;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    enum FlightState { WAIT_FCU = 0, ARM_OFFBOARD, TAKEOFF, HOVER, MOVE, ORBIT, LAND, DONE };

    bool flying() const
    {
        return flight_state_ == TAKEOFF || flight_state_ == HOVER ||
               flight_state_ == MOVE    || flight_state_ == ORBIT;
    }

    // ---------------- 参数解析 ----------------
    void parseWaypoints(ros::NodeHandle& pnh)
    {
        XmlRpc::XmlRpcValue wp;
        if (!pnh.getParam("waypoints", wp)) return;

        if (wp.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            // 形式一：roslaunch 的 <rosparam param="waypoints">[[2,0,0],[0,0,0.5]]</rosparam>
            if (wp.size() >= 3 && wp[0].getType() != XmlRpc::XmlRpcValue::TypeArray) {
                // 只写一个航点的简写：[2,0,0]
                Waypoint w;
                if (fillWaypoint(wp, w)) waypoints_.push_back(w);
            } else {
                for (int i = 0; i < wp.size(); ++i) {
                    if (wp[i].getType() != XmlRpc::XmlRpcValue::TypeArray) {
                        ROS_WARN("~waypoints[%d] 不是数组，已跳过", i);
                        continue;
                    }
                    Waypoint w;
                    if (fillWaypoint(wp[i], w)) waypoints_.push_back(w);
                }
            }
        } else if (wp.getType() == XmlRpc::XmlRpcValue::TypeString) {
            // 形式二：rosrun 的 _waypoints:="2,0,0;0,0,0.5"
            // 注意：命令行的 _param:= 传进来永远是字符串，不会做 YAML 解析，
            //       所以这里必须自己拆；roslaunch 的 <rosparam> 才会给真正的数组。
            parseWaypointsFromString(static_cast<std::string>(wp));
        } else {
            ROS_WARN("~waypoints 类型无法识别，已忽略");
            return;
        }

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            ROS_INFO("waypoint[%zu]: d=(%.2f, %.2f, %.2f) yaw=%s",
                     i, waypoints_[i].dx, waypoints_[i].dy, waypoints_[i].dz,
                     waypoints_[i].has_yaw ? "指定" : "自动朝运动方向");
        }
    }

    // 解析 "2,0,0;0,0,0.5;-2,0,0" 或 "[2,0,0]" 这类字符串写法
    void parseWaypointsFromString(const std::string& raw)
    {
        std::string s = raw;
        for (char& c : s) {
            if (c == '[' || c == ']') c = ' ';
        }
        if (s.find_first_not_of(" \t,;") == std::string::npos) return;   // 空串直接返回

        std::stringstream groups(s);
        std::string group;
        int parsed = 0;
        while (std::getline(groups, group, ';')) {
            std::stringstream nums(group);
            std::string tok;
            std::vector<double> v;
            while (std::getline(nums, tok, ',')) {
                if (tok.find_first_not_of(" \t") == std::string::npos) continue;
                try {
                    v.push_back(std::stod(tok));
                } catch (const std::exception&) {
                    ROS_WARN("~waypoints 里 '%s' 不是数字，已忽略", tok.c_str());
                }
            }
            if (v.size() < 3) {
                if (!v.empty()) ROS_WARN("航点至少需要 dx,dy,dz 三个数，已跳过该组");
                continue;
            }
            Waypoint w;
            w.dx = v[0];
            w.dy = v[1];
            w.dz = v[2];
            if (v.size() >= 4) { w.yaw = v[3]; w.has_yaw = true; }
            waypoints_.push_back(w);
            ++parsed;
        }
        if (parsed == 0) {
            ROS_WARN("~waypoints=\"%s\" 没解析出航点，格式应为 \"2,0,0;0,0,0.5\"", raw.c_str());
        }
    }

    bool fillWaypoint(const XmlRpc::XmlRpcValue& e, Waypoint& w)
    {
        if (e.getType() != XmlRpc::XmlRpcValue::TypeArray || e.size() < 3) {
            ROS_WARN("航点至少需要 [dx, dy, dz] 三个数，已跳过");
            return false;
        }
        w.dx = xmlToDouble(e[0]);
        w.dy = xmlToDouble(e[1]);
        w.dz = xmlToDouble(e[2]);
        if (e.size() >= 4) {
            w.yaw = xmlToDouble(e[3]);
            w.has_yaw = true;
        }
        return true;
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

    double distTo(double x, double y, double z) const
    {
        const double dx = current_pose_.pose.position.x - x;
        const double dy = current_pose_.pose.position.y - y;
        const double dz = current_pose_.pose.position.z - z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
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

        // 记录起飞点：既是返航点，也是环绕圆心和所有 waypoints 的原点
        center_x_ = current_pose_.pose.position.x;
        center_y_ = current_pose_.pose.position.y;
        ROS_INFO("FCU connected. origin = (%.2f, %.2f), ground z = %.2f",
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

        if (std::fabs(current_pose_.pose.position.z - height_) < takeoff_tol_) {
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
            if (!waypoints_.empty()) {
                wp_index_ = 0;
                wp_start_ = ros::Time::now();
                ROS_INFO("hover done -> %zu waypoint(s)", waypoints_.size());
                flight_state_ = MOVE;
            } else {
                ROS_INFO("hover done -> %s", laps_ > 0.0 ? "orbit" : "land");
                enterOrbitOrLand();
            }
        }
    }

    // ---------------- MOVE（前进/后退/平移/升降）----------------
    void move()
    {
        if (wp_index_ >= waypoints_.size()) {
            enterOrbitOrLand();
            return;
        }

        const Waypoint& w = waypoints_[wp_index_];
        const double tx = center_x_ + w.dx;
        const double ty = center_y_ + w.dy;
        const double tz = height_   + w.dz;   // dz 相对飞行高度

        // yaw：显式指定就用指定的；否则机头朝运动方向
        double yaw = 0.0;
        if (w.has_yaw) {
            yaw = w.yaw;
        } else {
            const double ydx = tx - current_pose_.pose.position.x;
            const double ydy = ty - current_pose_.pose.position.y;
            if (std::sqrt(ydx * ydx + ydy * ydy) > 0.2) yaw = std::atan2(ydy, ydx);
        }

        publishSetpoint(tx, ty, tz, yaw);

        const double d = distTo(tx, ty, tz);
        if (d < waypoint_tol_) {
            ROS_INFO("waypoint[%zu] reached (d=%.2f)", wp_index_, d);
            ++wp_index_;
            wp_start_ = ros::Time::now();
            if (wp_index_ >= waypoints_.size()) enterOrbitOrLand();
        } else if ((ros::Time::now() - wp_start_).toSec() > waypoint_timeout_) {
            ROS_WARN("waypoint[%zu] timeout (d=%.2f), skip", wp_index_, d);
            ++wp_index_;
            wp_start_ = ros::Time::now();
            if (wp_index_ >= waypoints_.size()) enterOrbitOrLand();
        }
    }

    void enterOrbitOrLand()
    {
        if (laps_ > 0.0) {
            orbit_start_ = ros::Time::now();
            ROS_INFO("-> orbit (R=%.1fm, v=%.1fm/s, %.1f lap)", radius_, speed_, laps_);
            flight_state_ = ORBIT;
        } else {
            ROS_INFO("orbit disabled (laps<=0) -> land");
            flight_state_ = LAND;
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

    double height_           = 2.0;
    double hover_time_       = 5.0;
    double waypoint_tol_     = 0.3;
    double waypoint_timeout_ = 20.0;
    double radius_           = 3.0;
    double speed_            = 1.0;
    double laps_             = 1.0;
    bool   clockwise_        = false;
    bool   face_tangent_     = true;
    double takeoff_tol_      = 0.25;
    double omega_            = 0.0;

    std::vector<Waypoint> waypoints_;
    size_t wp_index_ = 0;

    double center_x_ = 0.0;
    double center_y_ = 0.0;
    ros::Time phase_start_;
    ros::Time wp_start_;
    ros::Time orbit_start_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_orbit");
    OrbitController controller;
    controller.run();
    return 0;
}
