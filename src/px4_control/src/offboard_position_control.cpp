#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

/*
状态机

INIT
 |
ARM
 |
TAKEOFF
 |
HOVER
 |
MISSION
 |
RETURN
 |
LAND

*/

enum FlightState
{
    INIT = 0,
    ARM,
    TAKEOFF,
    HOVER,
    MISSION,
    RETURN_HOME,
    LAND
};

class OffboardController
{

public:

    OffboardController()
    {
        state = INIT;
        // 发布位置控制
        setpoint_pub =
            nh.advertise<geometry_msgs::PoseStamped>(
                "/mavros/setpoint_position/local",
                10);
        // PX4状态
        state_sub =
            nh.subscribe(
                "/mavros/state",
                10,
                &OffboardController::stateCallback,
                this);
        // 当前odom
        pose_sub =
            nh.subscribe(
                "/mavros/local_position/pose",
                10,
                &OffboardController::poseCallback,
                this);

        arming_client =
            nh.serviceClient<mavros_msgs::CommandBool>(
                "/mavros/cmd/arming");
        set_mode_client =
            nh.serviceClient<mavros_msgs::SetMode>(
                "/mavros/set_mode");
        ROS_INFO("controller init");

    }

    void run()
    {
        ros::Rate rate(50);
        /*
        必须先发送一段setpoint
        PX4要求:
        OFFBOARD之前
        先>2Hz发送
        */
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x=0;
        pose.pose.position.y=0;
        pose.pose.position.z=2;

        for(int i=0;i<100;i++)
        {
            setpoint_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        while(ros::ok())
        {
            switch(state)
            {
            case INIT:
                initState();
                break;

            case ARM:
                armState();
                break;

            case TAKEOFF:
                takeoffState();
                break;

            case HOVER:
                hoverState();
                break;

            case MISSION:
                missionState();
                break;

            case RETURN_HOME:
                returnState();
                break;

            case LAND:
                if(landState())
                {
                    return;
                }
                else break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
private:

    ros::NodeHandle nh;
    ros::Publisher setpoint_pub;

    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    mavros_msgs::State current_state;
//    nav_msgs::Odometry current_odom;
    geometry_msgs::PoseStamped current_pose;

    FlightState state;
    geometry_msgs::PoseStamped home;
    ros::Time hover_start;
    //--------------------
    // callback
    //--------------------
    void stateCallback(
        const mavros_msgs::State::ConstPtr& msg)
    {
        current_state=*msg;
    }

    void poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose=*msg;
    }
    //--------------------
    // INIT
    //--------------------
    void initState()
    {
        if(!current_state.connected)
        {
            ROS_INFO_THROTTLE(
                2,
                "waiting mavros");
            return;
        }

        if(current_pose.header.stamp.toSec()==0)
        {
            ROS_INFO_THROTTLE(
                2,
                "waiting odom");
            return;
        }
        home.pose =
            current_pose.pose;
        ROS_INFO("INIT finished");
        state=ARM;

    }
    //--------------------
    // ARM + OFFBOARD
    //--------------------

    void armState()
    {
        mavros_msgs::SetMode offb;
        offb.request.custom_mode="OFFBOARD";
        if(set_mode_client.call(offb))
        {
            if(offb.response.mode_sent)
            {
                ROS_INFO("OFFBOARD enabled");
            }
        }
        mavros_msgs::CommandBool arm;
        arm.request.value=true;
        if(arming_client.call(arm))
        {
            if(arm.response.success)
            {
                ROS_INFO("armed");
                state=TAKEOFF;
                ROS_WARN("state -- armed");
            }
        }
    }
    //--------------------
    // TAKEOFF
    //--------------------

    void takeoffState()
    {

//        ROS_WARN("state -- takeoff begin");
        geometry_msgs::PoseStamped target;

        target.pose.position.x =
            home.pose.position.x;
        target.pose.position.y =
            home.pose.position.y;
        target.pose.position.z =
            2.0;
//        ROS_WARN("state -- takeoff set point");
        setpoint_pub.publish(target);

        double z =
        current_pose.pose.position.z;
//        ROS_WARN("state -- height %f", z);
        if(z>1.8)
        {
            ROS_INFO("takeoff complete");
            hover_start=ros::Time::now();
            state=HOVER;
        }
    }
    //--------------------
    // HOVER
    //--------------------
    void hoverState()
    {
        geometry_msgs::PoseStamped target;
        target.pose.position =
            home.pose.position;

        target.pose.position.z=2;
        setpoint_pub.publish(target);

        if(
        ros::Time::now()-hover_start
        > ros::Duration(10))
        {

            ROS_INFO("start mission");
            state=MISSION;

        }
    }
    //--------------------
    // waypoint
    //--------------------

    void missionState()
    {

        geometry_msgs::PoseStamped target;
        // waypoint

        target.pose.position.x=5;

        target.pose.position.y=0;

        target.pose.position.z=2;

        setpoint_pub.publish(target);

        double dx =
        current_pose.pose.position.x-5;

        double dy =
        current_pose.pose.position.y;

        double dis =
        sqrt(dx*dx+dy*dy);

        if(dis<0.3)
        {
            ROS_INFO("mission finished");
            state=RETURN_HOME;
        }
    }
    //--------------------
    // return
    //--------------------

    void returnState()
    {
        geometry_msgs::PoseStamped target;
        target.pose.position =
            home.pose.position;
        target.pose.position.z=2;
        setpoint_pub.publish(target);

        double dx =
        current_pose.pose.position.x
        -home.pose.position.x;

        double dy =
        current_pose.pose.position.y
        -home.pose.position.y;

        if(sqrt(dx*dx+dy*dy)<0.3)
        {
            ROS_INFO("return home");
            state=LAND;
        }
    }
    //--------------------
    // LAND
    //--------------------

    int landState()
    {
        mavros_msgs::SetMode land;
        land.request.custom_mode="AUTO.LAND";
        if(set_mode_client.call(land))
        {
//            ROS_INFO("landing");
            double z =current_pose.pose.position.z;
//        ROS_WARN("state -- height %f", z);
            if(z<0.15)
            {
                ROS_INFO("land complete");
                return 1;
            }
            else {return 0;}
        }
    }
};

int main(int argc,char **argv)
{
    ros::init(
        argc,
        argv,
        "offboard_position_control");
    OffboardController controller;
    controller.run();
    return 0;

}