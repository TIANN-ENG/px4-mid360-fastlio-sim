#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""aruco_down_cam —— 下视相机 ArUco 检测与位姿解算节点。

订阅  /camera/image_raw + /camera/camera_info （Gazebo vision_cam 模型发布）
发布  ~pose         geometry_msgs/PoseStamped  相机系下的标识位姿
      ~debug_image  sensor_msgs/Image          带检测框与坐标轴的调试图

注意：
  1) 必须用系统 python3 运行（/usr/bin/python3）。conda 的 python3 里没有 cv2/cv_bridge。
  2) marker_length 必须与 aruco_pad 纹理的实际标识边长一致，默认 0.4 m。
  3) tvec[2] 是相机到标识平面的垂距，可直接作为 EKF 里的视觉深度观测 z_v，
     与激光测距 z_l 做比值即可在线估计尺度校正因子。
"""
import cv2
import numpy as np
import rospy
import tf.transformations as tft
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image


class ArucoDownCam(object):
    def __init__(self):
        self.marker_len = rospy.get_param('~marker_length', 0.4)
        dict_name = rospy.get_param('~dictionary', 'DICT_4X4_50')
        img_topic = rospy.get_param('~image_topic',
                                    '/iris_mid360/camera/image_raw')
        info_topic = rospy.get_param('~camera_info_topic',
                                     '/iris_mid360/camera/camera_info')
        self.axis_len = rospy.get_param('~axis_length', 0.1)
        # 位姿输出所用的坐标系：OpenCV 光学系，由 mavros_tf.launch 发布 TF
        self.pose_frame = rospy.get_param('~pose_frame', 'camera_optical_frame')

        if not hasattr(cv2.aruco, dict_name):
            rospy.logfatal('unknown aruco dictionary: %s', dict_name)
            raise SystemExit(1)
        self.dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
        self.params = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        self.pose_pub = rospy.Publisher('~pose', PoseStamped, queue_size=1)
        self.debug_pub = rospy.Publisher('~debug_image', Image, queue_size=1)
        rospy.Subscriber(info_topic, CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(img_topic, Image, self.cb_img, queue_size=1,
                         buff_size=2 ** 24)
        rospy.loginfo('aruco_down_cam ready: image=%s info=%s marker_length=%.3f m',
                      img_topic, info_topic, self.marker_len)

    def cb_info(self, msg):
        self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.D, dtype=np.float64)

    def cb_img(self, msg):
        if self.K is None:
            rospy.logwarn_throttle(5.0, 'waiting for camera_info ...')
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                      # noqa: BLE001
            rospy.logwarn_throttle(5.0, 'imgmsg_to_cv2 failed: %s', exc)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                  parameters=self.params)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_len, self.K, self.D)

        # 取最近的标识（降落场景下即目标板）
        idx = int(np.argmin([t[0][2] for t in tvecs]))
        rvec = np.asarray(rvecs[idx], dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvecs[idx], dtype=np.float64).reshape(3, 1)

        rot = np.eye(4)
        rot[:3, :3] = cv2.Rodrigues(rvec)[0]
        qx, qy, qz, qw = tft.quaternion_from_matrix(rot)

        pose = PoseStamped()
        pose.header = msg.header
        # cv2.aruco 的 tvec/rvec 是 OpenCV 光学系（x 右、y 下、z 沿光轴朝前）。
        # 图像自带的 frame_id 是 Gazebo 作用域名（含 "::"），不是合法 TF 帧，
        # 所以这里改用 mavros_tf.launch 发布的光学系帧名。
        pose.header.frame_id = self.pose_frame
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        self.pose_pub.publish(pose)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.aruco.drawAxis(frame, self.K, self.D, rvec, tvec, self.axis_len)
        cv2.putText(frame,
                    'id=%d  z=%.3f m  x=%.3f y=%.3f'
                    % (ids[idx][0], tvec[2], tvec[0], tvec[1]),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    rospy.init_node('aruco_down_cam')
    ArucoDownCam()
    rospy.spin()
