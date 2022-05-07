#!/usr/bin/env python
from inspect import Traceback
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from utils import TargetTracker, Utils, create_setpoint_message
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import time
import sys

class MarkerPose(TargetTracker, Utils):
    def __init__(self, raw_image_topic, init_point):
        super(MarkerPose, self).__init__()
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=5)
        self.range_sub = rospy.Subscriber("/iris/laser/range", LaserScan, self.get_range)
        self.sub1 = rospy.Subscriber(raw_image_topic, Image, self.retrieve_frame)
        self.sub3 = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_pose)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.get_uav_state)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
        self.raw_image = None
        self.u_o = 160 # camera center X
        self.v_o = 120 # camera center Y

        self.marker_ID = 70
        self.once = True
        self.AUTO_LAND = False

        # self.focal_length = 0.073340212942 # meters
        self.focal_length = 277.191356 # pixels
        self.max_land_alt = 5.4

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_frame = "map"
        self.camera_frame = "robot_camera_link"

        self.setpoint_msg = create_setpoint_message(init_point[0], init_point[1], init_point[2])
        self.marker_loc_list = []


    def retrieve_frame(self, data):
        self.raw_image = super().ros2cv2(data, encoding="bgr8") 

        box, id = super().detect_marker(self.raw_image)
        if np.any(id): 
            point = super().find_center(box, id, frame=self.raw_image, draw=True)
            self.u, self.v = point[self.marker_ID]
            self.prec_land_pipeline()
        else:
            point = {}
            try:
                if self.range <= self.max_land_alt and self.once == True:
                    result = self.set_mode(0, "AUTO.LAND")
                    self.AUTO_LAND = True
                    self.once = False
                    rospy.loginfo("PX4 mode set to AUTO.LAND")
            except:
                pass
        
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
            if self.range <= self.max_land_alt and self.once == True:
                result = self.set_mode(0, "AUTO.LAND")
                self.AUTO_LAND = True
                rospy.loginfo("PX4 mode set to AUTO.LAND")
        

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
        Y = (u - self.u_o) * depth / self.focal_length
        X = (v - self.v_o) * depth / self.focal_length
        Z = -depth
        return [X, Y, Z]
    
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
        unit_vector = np.multiply(unit_vector,np.array([1, 1, 0.3]))
        next_point = current+unit_vector
        return next_point

    def update_setpoint_msg(self, point):
        self.setpoint_msg.position.x = point[0]
        self.setpoint_msg.position.y = point[1]
        self.setpoint_msg.position.z = point[2]
        

    def loop(self):
        rospy.logwarn("ENGAGING PRECISION LANDING")
        time.sleep(2)
        while not rospy.core.is_shutdown():
            if self.AUTO_LAND == False:
                if(self.current_mode != "OFFBOARD"):
                    self.set_mode(0, "OFFBOARD")
                self.setpoint_pub.publish(self.setpoint_msg)
            try:
                if self.range < 0.15:
                    break
            except:
                pass
            rospy.rostime.wallsleep(1/10)
        rospy.loginfo("UAV successfully landed.")



