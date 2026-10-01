#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
import tf2_ros
import geometry_msgs.msg


class PX4OdomTF:

    def __init__(self):

        rospy.init_node("px4_odom_tf")

        self.br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self.pose_callback
        )


    def pose_callback(self,msg):

        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp =  rospy.Time.now()

        # Hector需要
        t.header.frame_id = "odom"

        # 飞机本体
        t.child_frame_id = "base_link"


        t.transform.translation.x = (
            msg.pose.position.x
        )

        t.transform.translation.y = (
            msg.pose.position.y
        )

        t.transform.translation.z = (
            msg.pose.position.z
        )


        t.transform.rotation = (
            msg.pose.orientation
        )


        self.br.sendTransform(t)



if __name__=="__main__":

    PX4OdomTF()

    rospy.spin()