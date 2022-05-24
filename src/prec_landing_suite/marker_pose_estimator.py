#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from utils import TargetTracker, Utils, create_setpoint_message
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import time
import sys

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("mav_companion_api"))

from src.vision_suite.camera_pose_estimator import CameraPose

class MarkerPose(TargetTracker, Utils, CameraPose):
    def __init__(self, 
            raw_image_topic, 
            cam_info, 
            distance_sensor_topic, 
            init_point, 
            marker_id, 
            update_coeffs=[1,1,0.3],
            ground_clearance=0.15):

        super(MarkerPose, self).__init__()
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=5)
        self.range_sub = rospy.Subscriber(distance_sensor_topic, LaserScan, self.get_range)
        self.sub1 = rospy.Subscriber(raw_image_topic, Image, self.retrieve_frame)
        self.sub_cam_info = rospy.Subscriber(cam_info, CameraInfo, self.callback_cam_info)
        self.sub3 = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_pose)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.get_uav_state)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
        self.raw_image = None
        self.u_o = 160 # None camera center X ###############
        self.v_o = 120 # None camera center Y ###############

        self.marker_ID = marker_id
        self.once = True
        self.AUTO_LAND = False
        self.max_land_alt = 5.4

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_frame = "map"
        self.camera_frame = "robot_camera_link"
        self.distance_sensor_link = None ##################################

        self.setpoint_msg = create_setpoint_message(init_point[0], init_point[1], init_point[2])
        self.marker_loc_list = []

        self.update_coeffs = update_coeffs # List in meters
        self.ground_clearance = ground_clearance

    def callback_cam_info(self, data):

        self.distortion_params = data.D
        self.camera_intrinsic = data.K
        self.cam_mtx = np.reshape(self.camera_intrinsic, (3,3))
        self.cam_mtx[0,2] = self.u_o
        self.cam_mtx[1,2] = self.v_o
        self.cx = self.camera_intrinsic[2]
        self.cy = self.camera_intrinsic[5]
        self.h = data.height
        self.w = data.width
        self.projection_matrix = data.P
        self.proj_mtx = np.reshape(self.projection_matrix, (3,4))
        self.fx = data.K[0]
        self.fy = data.K[4]


    def retrieve_frame(self, data):
        self.raw_image = super().ros2cv2(data, encoding="bgr8") 
        self.box, self.id = super().detect_marker(self.raw_image)

        try:
            if self.range > self.max_land_alt:
                if (np.any(self.id) and (self.marker_ID in self.id)): 
                    point = super().find_center(self.box, self.id, frame=self.raw_image, draw=True)
                    self.u, self.v = point[self.marker_ID]
                    self.prec_land_pipeline()
                else: 
                    rospy.loginfo("Marker with ID \'{0}\' is not detected".format(self.marker_ID))
                    # Here goes the code for the case where
                    # the drone does not detect any marker 
                    # and it is not anywhere close to the ground.
                    # Ideally the framework should identify flat
                    # ground suitable for landing and make a slow 
                    # descent. 
                    pass
            else:
                # Before setting to AUTO.LAND, check whether
                # the ground beneath is suitable for landing.
                # Also, check if any portion of the marker is
                # visible underneath the UAV.
                if self.once == True: 
                    result = self.set_mode(0, "AUTO.LAND")
                    rospy.loginfo("PX4 mode set to AUTO.LAND")
                    self.AUTO_LAND = True
                    self.once = False
        except:
            rospy.logerr("Variable \'self.range\' not yet defined.")
        
        #### Comment the line below while running on offboard CPU ####
        super().display_image(["iris_raw_image"], [self.raw_image])
        

    def prec_land_pipeline(self):
        cam_frame_X, cam_frame_Y, cam_frame_Z = self.pose_estimator_rel_to_cam(depth=self.range, u=self.u, v=self.v)
        tf_map_to_cam = self.find_transformation()
        if tf_map_to_cam != None:
            point_wrt_map = self.transform_point(tf_map_to_cam, Point(cam_frame_X, cam_frame_Y, cam_frame_Z))
            self.marker_loc_list.append(point_wrt_map)
            next_point = self.compute_path_vector([self.current_x, self.current_y, self.current_z], point_wrt_map)
            self.update_setpoint_msg(next_point)
            self.mean, self.std_error, self.std_per_error, self.std_dev = super().get_mean_uncertainty(np.array(self.marker_loc_list))
            rospy.loginfo(
                "MARKER POSITION ESTIMATE\nMean:{0} | Uncertainty (m):{1}".format(self.mean, self.std_dev))
        else:
            rospy.logerr(
                "Unable to find transform between \"{}\" and \"{}\"".format(self.map_frame, self.camera_frame)
                )
        

    def get_pose(self, data):
        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z
        self.current_rotX = data.pose.orientation.x
        self.current_rotY = data.pose.orientation.y
        self.current_rotZ = data.pose.orientation.z
        self.current_rotW = data.pose.orientation.w

    def get_range(self, data):
        self.range = data.ranges[0]

    def get_uav_state(self, data):
        self.conn_status = data.connected
        self.current_mode = data.mode
        self.armed = data.armed

    def pose_estimator_rel_to_cam(self, depth, u, v):
        # Y = (u - self.u_o) * depth / self.focal_length
        # X = (v - self.v_o) * depth / self.focal_length
        # Z = -depth

        points, pixels = super().undistort_point(
            [[u, v, depth]], self.u_o, self.v_o, self.fx, self.fy, self.distortion_params)
        vertex_pts = np.squeeze(np.array(self.box, dtype=np.float32))
        vertex_pts_3d = np.append(vertex_pts, np.array([[depth,depth,depth,depth]]).T, axis=1)
    
        # corrected_pts, corrected_pixels = super().undistort_point(
        #     vertex_pts_3d, self.u_o, self.v_o, self.fx, self.fy, self.distortion_params)
        # #print(vertex_pts, corrected_pixels)

        # rvec, tvec = super().estimate_pose(
        #     np.array(corrected_pixels, dtype=np.float32), publish=False, cam_mtx=self.cam_mtx, distortion_params=self.distortion_params)
        # tvec = np.squeeze(np.array(tvec).T)
        # X_est, Y_est, Z_est = tvec 

        return [points[0][0], points[0][1], -depth]
    
    def find_transformation(self):
        try:
            transformation = self.tf_buffer.lookup_transform(self.map_frame, self.camera_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from {0} to {1}'.format(self.camera_frame, self.map_frame))
            return None

        return transformation      

    def transform_point(self, transformation, point_wrt_camera_frame):
        point_wrt_map_frame = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=point_wrt_camera_frame), 
            transformation
            ).point
        
        return [point_wrt_map_frame.x, point_wrt_map_frame.y, point_wrt_map_frame.z]

    def compute_path_vector(self, current, goal):
        vector = np.array(goal) - np.array(current)
        unit_vector = vector / np.linalg.norm(vector)
        unit_vector = np.multiply(unit_vector,np.array(self.update_coeffs))
        next_point = current+unit_vector
        return next_point

    def update_setpoint_msg(self, point):
        self.setpoint_msg.position.x = point[0]
        self.setpoint_msg.position.y = point[1]
        self.setpoint_msg.position.z = point[2]
        

    def loop(self):
        rospy.logwarn("ENGAGING PRECISION LANDING")
        while not rospy.core.is_shutdown():
            if self.AUTO_LAND == False:
                try:
                    if(self.current_mode != "OFFBOARD"):
                        self.set_mode(0, "OFFBOARD")
                except:
                    pass
                self.setpoint_pub.publish(self.setpoint_msg)
            try:
                if self.range < self.ground_clearance:
                    break
            except:
                pass
            rospy.rostime.wallsleep(1/10)
        rospy.loginfo("UAV successfully landed.")