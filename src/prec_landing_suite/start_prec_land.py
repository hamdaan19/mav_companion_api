#!/usr/bin/env python3
import rospy
from marker_pose_estimator import MarkerPose
from rospy.core import is_shutdown
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
import time
from mavros_msgs.srv import SetMode, CommandBool
from utils import create_setpoint_message, get_distance, get_uncertain_init_point

GOTO_INIT_POINT = True
RANDOM_POINT = False
ENGAGE_PREC_LANDING = True

RAW_IMAGE_TOPIC = "/iris/usb_cam/image_raw"
CAM_INFO = "/iris/usb_cam/camera_info"
DIST_SENSOR_TOPIC = "/iris/laser/range"
MARKER_ID = 70
# GROUND_CLEARANCE = 0.3 # 0.26
# UPDATE_COEFFICIENTS = [1,1,0.3]

PUB_TOPIC_GOTO_POINT = "/mavros/setpoint_raw/local"
UAV_STATE_TOPIC = "mavros/state"
INIT_POINT = [3, 3, 20] 

def state_callback(data):
    global conn_status 
    global current_mode
    global armed
    conn_status = data.connected
    current_mode = data.mode
    armed = data.armed

def pose_callback(data):
    global posX
    global posY
    global posZ
    posX = data.pose.position.x
    posY = data.pose.position.y
    posZ = data.pose.position.z

def main():
    rate = rospy.Rate(20)
    while(conn_status == False and (not rospy.is_shutdown())):
        rospy.loginfo("Waiting for connection...")
        rate.sleep()

    setpoint_msg.position.x = posX
    setpoint_msg.position.y = posY
    setpoint_msg.position.z = posZ
    
    if GOTO_INIT_POINT == True:

        if RANDOM_POINT == True:
            INIT_POINT[0], INIT_POINT[1] = get_uncertain_init_point([0, 0], radius=9)

        setpoint_msg.position.x = INIT_POINT[0]
        setpoint_msg.position.y = INIT_POINT[1]
        setpoint_msg.position.z = INIT_POINT[2]

        arm_quad(True)
        rospy.loginfo("Vehicle armed")
        res1 = set_mode(0, "AUTO.TAKEOFF")
        rospy.loginfo("Mode set to AUTO.TAKEOFF")

    for i in range(10):
        pub.publish(setpoint_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC_GOTO_POINT} - {i}")
        rate.sleep()
        if (rospy.is_shutdown()):
            break

    arm_quad(True)
    rospy.loginfo("Vehicle armed")
    res2 = set_mode(0, "OFFBOARD")
    rospy.loginfo("Mode set to OFFBOARD")

    if GOTO_INIT_POINT == True:  

        rospy.loginfo("Approaching Initial Point {}".format(INIT_POINT))
        while not rospy.core.is_shutdown():
            pub.publish(setpoint_msg)
            if get_distance(INIT_POINT, [posX, posY, posZ]) <= 0.25:
                rospy.loginfo("Initial Point Reached.")
                break
            rospy.rostime.wallsleep(1/10)

    if ENGAGE_PREC_LANDING == True:
        start_prec_land = MarkerPose(
            raw_image_topic=RAW_IMAGE_TOPIC, 
            cam_info=CAM_INFO, 
            distance_sensor_topic=DIST_SENSOR_TOPIC,
            init_point=INIT_POINT,
            marker_id=MARKER_ID,
            #ground_clearance=GROUND_CLEARANCE,
            #update_coeffs=UPDATE_COEFFICIENTS,
            )
        start_prec_land.loop()

if __name__ == '__main__':
    rospy.init_node('precision_landing_node', anonymous=False)
    pub = rospy.Publisher(PUB_TOPIC_GOTO_POINT, PositionTarget, queue_size=5)
    state_sub = rospy.Subscriber(UAV_STATE_TOPIC, State, state_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arm_quad = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    global setpoint_msg
    setpoint_msg = create_setpoint_message()

    time.sleep(2)
    main()