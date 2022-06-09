#!/usr/bin/env python3
import rospy
import time
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from marker_pose_estimator import MarkerPose
from utils import create_setpoint_message, TargetTracker, Utils

RAW_IMAGE_TOPIC = "/uav/downward_cam/image_raw"
CAM_INFO = "/uav/downward_cam/camera_info"
DIST_SENSOR_TOPIC = "/mavros/distance_sensor/lightware_lw20_pub"
MARKER_ID = 320
GROUND_CLEARANCE = 0.3 # 0.26
#UPDATE_COEFFICIENTS = [0.3,0.3,0.75]

PUB_TOPIC_GOTO_POINT = "/mavros/setpoint_raw/local"
UAV_STATE_TOPIC = "mavros/state"
INIT_POINT = [-2.4, -6.6, 3] 
ENGAGE_PREC_LANDING = True
marker_detected = False
HOLD_CURRENT_POSITION = True

def state_callback(data):
    global conn_status
    global current_mode
    global armed
    conn_status = data.connected
    current_mode = data.mode
    armed = data.armed


def main():
    rate = rospy.Rate(20)
    while(conn_status == False and (not rospy.core.is_shutdown())):
        rospy.loginfo("Waiting for connection...")
        rospy.rostime.wallsleep(1/10)

    setpoint_msg.position.x = INIT_POINT[0]
    setpoint_msg.position.y = INIT_POINT[1]
    setpoint_msg.position.z = INIT_POINT[2]

    while not rospy.core.is_shutdown():
        if HOLD_CURRENT_POSITION == True:
            try: 
                setpoint_msg.position.x = posX
                setpoint_msg.position.y = posY
                setpoint_msg.position.z = posZ
            except:
                rospy.logwarn("posX, posY, posZ may not be defined. Going to INIT_POINT.")
                #setpoint_msg.position.x = INIT_POINT[0]
                #setpoint_msg.position.y = INIT_POINT[1]
                #setpoint_msg.position.z = INIT_POINT[2]
                time.sleep(3)

        pub.publish(setpoint_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC_GOTO_POINT}")
        if marker_detected:
            rospy.loginfo("Marker detected")
        else:
            rospy.loginfo("NO Marker")
            
        if ((current_mode == "OFFBOARD") & ENGAGE_PREC_LANDING ):
            break
        elif (current_mode == "OFFBOARD"):
            HOLD_CURRENT_POSITION = False

        rospy.rostime.wallsleep(1/10)
    
    if ENGAGE_PREC_LANDING:
        start_prec_land = MarkerPose(
            raw_image_topic=RAW_IMAGE_TOPIC, 
            cam_info=CAM_INFO, 
            distance_sensor_topic=DIST_SENSOR_TOPIC,
            init_point=INIT_POINT,
            marker_id=MARKER_ID,
            ground_clearance=GROUND_CLEARANCE,
            #update_coeffs=UPDATE_COEFFICIENTS,
            )
        start_prec_land.loop()


def get_frame(data):
    global marker_detected
    raw_image = Utils().ros2cv2(data, encoding="bgr8")
    box, id = TargetTracker().detect_marker(image=raw_image)
    marker_detected = np.any(id)

def pose_cb(data):
    global posX, posY, posZ
    posX = data.pose.position.x
    posY = data.pose.position.y
    posZ = data.pose.position.z

if __name__ == "__main__":
    rospy.init_node("Wait_for_offboard_node", anonymous=False)
    state_sub = rospy.Subscriber(UAV_STATE_TOPIC, State, state_callback)
    pub = rospy.Publisher(PUB_TOPIC_GOTO_POINT, PositionTarget, queue_size=5)
    sub_cam = rospy.Subscriber(RAW_IMAGE_TOPIC, Image, get_frame)
    sub_pose = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    setpoint_msg = create_setpoint_message()

    time.sleep(2)
    main()
