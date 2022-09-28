#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State, PositionTarget
from marker_pose_estimator import MarkerPose
from utils import create_setpoint_message

RAW_IMAGE_TOPIC = "/iris/usb_cam/image_raw"
CAM_INFO = "/iris/usb_cam/camera_info"
DIST_SENSOR_TOPIC = "/iris/laser/range"
MARKER_ID = 70
GROUND_CLEARANCE = 0.3 # 0.26
# UPDATE_COEFFICIENTS = [1,1,0.3]

PUB_TOPIC_GOTO_POINT = "/mavros/setpoint_raw/local"
UAV_STATE_TOPIC = "mavros/state"
INIT_POINT = [3, 3, 3] 

ENGAGE_PREC_LANDING = True

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
    while(conn_status == False and (not rospy.core.is_shutdown())):
        rospy.loginfo("Waiting for connection...")
        rospy.rostime.wallsleep(1/10)

    setpoint_msg.position.x = INIT_POINT[0]
    setpoint_msg.position.y = INIT_POINT[1]
    setpoint_msg.position.z = INIT_POINT[2]

    while not rospy.core.is_shutdown():
        pub.publish(setpoint_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC_GOTO_POINT} - {i}")
        if ((current_mode == "OFFBOARD") & ENGAGE_PREC_LANDING ):
            break
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





if __name__ == "__main__":
    rospy.init_node("Wait_for_offboard_node", anonymous=False)
    state_sub = rospy.Subscriber(UAV_STATE_TOPIC, State, state_callback)
    pub = rospy.Publisher(PUB_TOPIC_GOTO_POINT, PositionTarget, queue_size=5)

    setpoint_msg = create_setpoint_message()

    main()

    rospy.spin()