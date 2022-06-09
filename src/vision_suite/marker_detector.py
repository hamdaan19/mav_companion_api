#!/usr/bin/env python3
import cv2
import rospy
import cv2.aruco as aruco
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import math
import rospkg
import sys

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("mav_companion_api"))

from src.prec_landing_suite.utils import Utils, TargetTracker

MARKER_ID = 320
bridge = CvBridge()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # raw_img = bridge.imgmsg_to_cv2(data, "bgr8")
    box, id = TargetTracker().detect_marker(image=cv_image)

    # undistorted_img, roi_img = Utils().undistort(
    #     frame=raw_img, cam_mtx=cam_mtx, dist=np.array(distortion_params, dtype=np.float32), frame_size=(w,h)
    # )
    if np.any(id): 
        print("marker detected.")
        points = TargetTracker().find_center(boxes=box, ids=id, frame=cv_image, draw=True)
        print(points)
        x, y = points[MARKER_ID]
        pts, pixels = TargetTracker().undistort_point(points=[[x, y, 0.5]], cx=cx, cy=cy, fx=fx, fy=fy, distortion_params=distortion_params)
        U, V = pixels[0]
        X, Y = pts[0]
        print(X, Y)
        U, V = round(U), round(V)
        pt = np.array([x, y], dtype=np.float64)
        undist_pts = cv2.undistortPoints(pt, cam_mtx, distortion_params, P=proj_mtx)
        undist_pts = (round(undist_pts[0][0][0]), round(undist_pts[0][0][1]))
        # cv2.circle(undistorted_img, undist_pts, 2, (0, 0, 255), 2)
        # cv2.circle(undistorted_img, (U,V), 2, (0, 255, 0), 2)
    else:
        print("marker not detected")

    
    # cv2.imshow("Undistorted Image", undistorted_img)
    # cv2.imshow("Marker Detector", cv_image)
    # cv2.waitKey(1)

def callback_cam_info(data):
    global distortion_params
    global camera_intrinsic, cam_mtx
    global cx, cy
    global h, w
    global projection_matrix, proj_mtx
    global fx, fy

    distortion_params = data.D
    camera_intrinsic = data.K
    cam_mtx = np.reshape(camera_intrinsic, (3,3))
    cx = camera_intrinsic[2]
    cy = camera_intrinsic[5]
    h = data.height
    w = data.width
    projection_matrix = data.P
    proj_mtx = np.reshape(projection_matrix, (3,4))
    fx = data.K[0]
    fy = data.K[4]


if __name__ == "__main__":
    rospy.init_node("marker_detector")
    rospy.Subscriber("/uav/downward_cam/image_raw", Image, callback)
    rospy.Subscriber("/uav/downward_cam/camera_info", CameraInfo, callback_cam_info)

    rospy.spin()
