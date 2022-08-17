#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import yaml


if __name__ == "__main__":
    bridge = CvBridge()
    vid = cv2.VideoCapture(2) # define a video capture object

    rospy.init_node("down_cam_node", anonymous=False)
    stream_pub = rospy.Publisher("uav/downward_cam/image_raw", Image, queue_size=5)
    stream_param_pub = rospy.Publisher("uav/downward_cam/camera_info", CameraInfo, queue_size=5)

    rate = rospy.Rate(30)

    with open("/home/hamdaan/ROS/px4_ws/src/mav_companion_api/data/econ_cam1.yaml", "r") as file_handle:
        calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    if calib_data["projection_matrix"]["data"] != []:
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]

    while not rospy.is_shutdown():
        ret, frame = vid.read()
        dim = (480, 300)
        if ret:
            frame_resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
            img_msg = bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            stream_pub.publish(img_msg)
            stream_param_pub.publish(camera_info_msg)
        rate.sleep()

    vid.release()
