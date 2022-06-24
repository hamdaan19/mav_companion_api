#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from mavros_msgs.msg import PositionTarget
import math, random

class TargetTracker(object):
    def __init__(self):
        super(TargetTracker, self).__init__()
        pass

    def detect_marker(self, image, marker_size=4, total_markers=50):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        method = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
        arucoDict = aruco.Dictionary_get(method)
        arucoParam = aruco.DetectorParameters_create()
        box, id, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)

        return (box, id)

    def find_center(self, boxes, ids, frame, draw=False):
        points = {}

        for i in range(len(ids)): # Length of ids gives me no. of boxes detected inside the frame
            pts = np.reshape(boxes[i], (4, 2))

            # calculating center point
            pt1 = pts[0].astype('int32')
            x1, y1 = pt1[0], pt1[1]

            pt2 = pts[2].astype('int32')
            x2, y2 = pt2[0], pt2[1]

            pt3 = pts[1].astype('int32')
            x3, y3 = pt3[0], pt3[1]

            pt4 = pts[3].astype('int32')
            x4, y4 = pt4[0], pt4[1]

            slope1 = (y2-y1)/(x2-x1)
            slope2 = (y4-y3)/(x4-x3)

            X = int(((slope1*x1)-(slope2*x3)+y3-y1)/(slope1-slope2))
            Y = int(slope1*(X-x1)+y1)

            if draw:
                cv2.line(frame, tuple(pt1), tuple(pt2), (255, 0, 0), 2)
                cv2.line(frame, tuple(pt3), tuple(pt4), (255, 0, 0), 2)
                cv2.circle(frame, (X, Y), 2, (0, 0, 255), 2)

            temp = dict([ (ids[i][0], (X,Y)) ])
            points.update(temp)

        return points

    def draw_axis(self, frame, center_pt, axis_pts, wt=5):
        cv2.line(frame, center_pt, tuple(axis_pts[0].ravel()), (255,0,0), wt)
        cv2.line(frame, center_pt, tuple(axis_pts[1].ravel()), (0,255,0), wt)
        cv2.line(frame, center_pt, tuple(axis_pts[2].ravel()), (0,0,255), wt)

    def undistort_point(self, points, cx, cy, fx, fy, distortion_params):
        corrected_cartesian_pts = []
        corrected_pixel_pts = []

        for point in points:
            u, v, depth = point

            y_ = (u - cx)*depth/fx ## Interchanged x_ and y_
            x_ = (v - cy)*depth/fy 

            r = math.sqrt(x_**2 + y_**2)
            k1, k2, p1, p2, k3 = distortion_params

            x_rad = x_ * (1 + k1*(r**2) + k2*(r**4) + k3*(r**6))
            y_rad = y_ * (1 + k1*(r**2) + k2*(r**4) + k3*(r**6))

            # Tangential Distortion Effect
            del_x_tan = 2*p1*x_*y_ + p2*((r**2) + 2*(x_**2))
            del_y_tan = p1*(r**2 + 2*(y_**2)) + 2*p2*x_*y_ 

            # # Plumb Bob Distortion Correction
            x_corrected = x_rad + del_x_tan
            y_corrected = y_rad + del_y_tan
            v_corrected = (x_corrected*fx)/depth + cy # Interchanged u_corrected and v_corrected
            u_corrected = (y_corrected*fy)/depth + cx # and cx and cy

            corrected_cartesian_pts.append([x_corrected, y_corrected])
            corrected_pixel_pts.append([u_corrected, v_corrected])

        return (corrected_cartesian_pts, corrected_pixel_pts)


class Utils(object):
    def __init__(self):
        super(Utils, self).__init__()
        self.bridge = CvBridge()

    def ros2cv2(self, image_message, encoding="passthrough"):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(image_message, encoding)
            return cv_image
        except CvBridgeError as e:
            print(e)

    def cv2ros(self, array, encoding="bgr8"):
        pass

    def display_image(self, names, frames):
        for name, frame in zip(names, frames):
            cv2.imshow(name, frame)
        cv2.waitKey(1)

    def get_mean_uncertainty(self, arr):
        mean = np.mean(arr, axis=0)
        mean = np.around(mean, decimals=3) #millimeter accuracy
        std = np.std(arr, axis=0)
        uncertainty = std / math.sqrt(arr.shape[0])
        percent_uncertainty = np.true_divide(uncertainty, mean)*100
        percent_uncertainty = np.absolute(percent_uncertainty)
        return mean, np.around(uncertainty, decimals=3), percent_uncertainty, np.around(std, decimals=3)

    def undistort(self, frame, cam_mtx, dist, frame_size):
        # frame_size should be a tuple (width, height)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist, frame_size, 1, frame_size)
        dst = cv2.undistort(frame, cam_mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        dst_roi = dst[y:y+h, x:x+w]
        return dst, dst_roi

    def adjust_brightness_contrast(self, frame, alpha=1, beta=10):
        result = cv2.addWeighted(frame, alpha, np.zeros(frame.shape, frame.dtype), 0, beta)
        return result

    def adjust_gamma(self, frame, gamma=1.0):
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")
        # apply gamma correction using the lookup table
        return cv2.LUT(frame, table)



def create_setpoint_message(X=0, Y=0, Z=0):
    setpoint_msg = PositionTarget()
    setpoint_msg.header.stamp = rospy.Time.now()
    setpoint_msg.header.frame_id = "map"

    setpoint_msg.coordinate_frame = 1
    setpoint_msg.type_mask = 0

    setpoint_msg.position.x = X
    setpoint_msg.position.y = Y
    setpoint_msg.position.z = Z

    setpoint_msg.velocity.x = 0
    setpoint_msg.velocity.y = 0
    setpoint_msg.velocity.z = 0

    setpoint_msg.acceleration_or_force.x = 0
    setpoint_msg.acceleration_or_force.y = 0
    setpoint_msg.acceleration_or_force.z = 0

    setpoint_msg.yaw = 0
    setpoint_msg.yaw_rate = 0

    return setpoint_msg


def get_distance(pointG, pointC):
    goalX, goalY, goalZ = pointG
    posX, posY, posZ = pointC
    distance = math.sqrt((goalX-posX)**2 + (goalY-posY)**2 + (goalZ-posZ)**2)
    return distance

def get_uncertain_init_point(point, radius):
    pointX, pointY = point
    radius_new = random.uniform(0, radius)
    theta = random.uniform(0, 6.28)
    X = radius_new * math.cos(theta)
    Y = radius_new * math.sin(theta)
    pointX_new = pointX + X
    pointY_new = pointY + Y

    return pointX_new, pointY_new
