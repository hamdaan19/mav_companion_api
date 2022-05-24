#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from marker_pose_estimator import MarkerPose

RAW_IMAGE_TOPIC = "/iris/usb_cam/image_raw"
CAM_INFO = "/iris/usb_cam/camera_info"
DIST_SENSOR_TOPIC = "/iris/laser/range"
MARKER_ID = 70
GROUND_CLEARANCE = 0.3 # 0.26
# UPDATE_COEFFICIENTS = [1,1,0.3]

PUB_TOPIC_GOTO_POINT = "/mavros/setpoint_raw/local"
UAV_STATE_TOPIC = "mavros/state"
INIT_POINT = [3, 3, 20] 

def state_callback(data):
    conn_status = data.connected
    current_mode = data.mode
    armed = data.armed

    if current_mode == "OFFBOARD":
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

    rospy.spin()