#!/usr/bin/env python3
from matplotlib.pyplot import connect
import rospy
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from mavros_msgs.msg import Waypoint, CommandCode, State
import numpy as np
import time

def state_callback(data):
    global connected 
    global current_mode
    global armed
    connected = data.connected
    current_mode = data.mode
    armed = data.armed

def get_waypoint_msg(x_lat, y_long, z_alt):
    wp = Waypoint()
    wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp.command = CommandCode.NAV_TAKEOFF
    wp.is_current = False
    wp.autocontinue = True 
    wp.x_lat = x_lat
    wp.y_long = y_long
    wp.z_alt = z_alt
    wp.param1 = 0.5 # Hold time
    wp.param2 = 0.1 # Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
    wp.param3 = 0   # 0 to pass through the WP, if > 0 radius to pass by WP. 
    wp.param4 = 0   # Desired yaw angle at waypoint (rotary wing). 
    # Check https://mavlink.io/en/messages/common.html (MAVLink Commands (MAV_CMD)) for more info. 

    return wp

def main():
    rate = rospy.Rate(20)
    waypoints = [] 
    rospy.loginfo("Generating waypoints")
    for z in np.arange(2.5, 12.5, 0.75):
        waypoints.append(get_waypoint_msg(x_lat=47.3962527, y_long=8.5467917, z_alt=z))

    while(not connected):
        rospy.loginfo("Waiting for connection...")
        rate.sleep()

    while (not armed):
        arm_quad(True)
        rospy.loginfo("Trying to arm.")
        rate.sleep()
    rospy.loginfo("Vehicle armed.")
    set_mode(0, "AUTO.TAKEOFF")

    if (current_mode != "AUTO.MISSION"):
        success = set_mode(0, "AUTO.MISSION")
        rospy.loginfo(f"AUTO.MISSION status: {success}")

    rospy.wait_for_service("mavros/mission/push")
    result = waypoint_srv(0, waypoints)
    
    if (result.success != True):
        rospy.logerr("Sending waypoints failed.")
    else:
        rospy.loginfo("Waypoints successfully published")

    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("waypoint_publisher", anonymous=False)
    waypoint_srv = rospy.ServiceProxy("mavros/mission/push", WaypointPush)
    arm_quad = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)
    time.sleep(2)
    rospy.loginfo("Entering main loop.")
    main()