#!/usr/bin/env python
import cv2
import rospy
import sys
import rospkg
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
import numpy as np
from tf.transformations import quaternion_from_euler

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("mav_companion_api"))

from src.prec_landing_suite.utils import TargetTracker, Utils


class CameraPose(TargetTracker, Utils):
    def __init__(self, sqr_len=1, image_topic="uav/downward_cam/image_raw", cam_info_topic="uav/downward_cam/camera_info"):
        super(CameraPose, self).__init__()
        self.image_topic = image_topic
        self.cam_info_topic = cam_info_topic
        self.sqr_len = sqr_len
        self.axis_len = 9

    def callback_image(self, data):
        self.cv_image = super().ros2cv2(data, encoding="bgr8")
        box, id = super().detect_marker(image=self.cv_image)
        vertex_pts = np.squeeze(np.array(box, dtype=np.float32))
        #print(vertex_pts)
        if np.any(id): 
            self.center_points = super().find_center(boxes=box, ids=id, frame=self.cv_image, draw=False)
            #vertex_pts = np.append(vertex_pts, np.zeros((4,1), dtype=np.float32), axis=1)
            r_, t_ = self.estimate_pose(vertex_pts, project_3d=True)
        cv2.imshow("Image", self.cv_image)
        cv2.waitKey(1)

    def callback_cam_info(self, data):
        self.distortion_params = data.D
        self.camera_intrinsic = data.K
        self.cam_mtx = np.reshape(self.camera_intrinsic, (3,3))
        self.cx = self.camera_intrinsic[2]
        self.cy = self.camera_intrinsic[5]

    def get_object_points(self):
        pts = [
            [-self.sqr_len/2, self.sqr_len/2, 0],
            [self.sqr_len/2, self.sqr_len/2, 0],
            [self.sqr_len/2, -self.sqr_len/2, 0],
            [-self.sqr_len/2, -self.sqr_len/2, 0]
        ]
        pts = np.array(pts, dtype=np.float32)
        return pts

    def estimate_pose(self, img_pts, project_3d=False, publish=True, **kwargs):
        obj_pts = self.get_object_points()
        #print(self.cam_mtx)
        if kwargs:
            self.cam_mtx = kwargs["cam_mtx"]
            self.distortion_params = kwargs["distortion_params"]

        ret, rvec, tvec = cv2.solvePnP(
            objectPoints=obj_pts,
            imagePoints=img_pts,
            cameraMatrix=self.cam_mtx,
            distCoeffs=np.array(self.distortion_params),
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if project_3d:
            axis = np.float32([[self.axis_len, 0, 0], [0, self.axis_len, 0], [0, 0, self.axis_len]]).reshape(-1, 3)
            proj_pts, jacob = cv2.projectPoints(axis, rvec, tvec, self.cam_mtx, np.array(self.distortion_params))
            id = list(self.center_points.keys())[0] # Getting the key value (marker ID) of the first marker
            center_pt = self.center_points[id] # Getting the center point pixel coordinates
            super().draw_axis(self.cv_image, center_pt, proj_pts)

        if publish:
            pose_msg = Pose()
            pose_msg.position.x = tvec[0][0]
            pose_msg.position.y = tvec[1][0]
            pose_msg.position.z = tvec[2][0]

            q = quaternion_from_euler(rvec[0][0], rvec[1][0], rvec[2][0])

            pose_msg.orientation.x = q[0]
            pose_msg.orientation.y = q[1]
            pose_msg.orientation.z = q[2]
            pose_msg.orientation.w = q[3]

            self.camera_pose_pub.publish(pose_msg)
        return [rvec, tvec]

    def loop(self):
        rospy.Subscriber(self.image_topic, Image, self.callback_image)
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.callback_cam_info)
        self.camera_pose_pub = rospy.Publisher("uav/downward_cam/cam_to_marker_pose", Pose, queue_size=5)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("camera_pose_estimator", anonymous=True)

    pose_tracker = CameraPose()
    pose_tracker.loop()
