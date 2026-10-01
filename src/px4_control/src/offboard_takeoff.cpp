#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

int main(int argc,char **argv)
{

    ros::init(argc,argv,"offboard_takeoff");

    ros::NodeHandle nh;
    ros::Publisher setpoint_pub =
        nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local",10);

    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>
        ("/mavros/cmd/arming");

    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;
    //目标高度
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.0;

    /*
      关键:
      先发送一段时间setpoint
    */
    for(int i=0;i<100;i++)
    {
        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode="OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;

    arm_cmd.request.value=true;

    //进入offboard

    if(set_mode_client.call(offb_set_mode)
       &&
       offb_set_mode.response.mode_sent)
    {
        ROS_INFO("OFFBOARD enabled");
    }
    //解锁
    if(arming_client.call(arm_cmd)
       &&
       arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
    }

    while(ros::ok())
    {
        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
