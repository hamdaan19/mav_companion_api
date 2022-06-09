#!/usr/bin/env python3
from matplotlib.pyplot import connect
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
import numpy as np
import time
import math

WAYPOINT_RADIUS = 0.2
PUB_TOPIC = "/mavros/setpoint_position/local"

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

def generate_waypoints():
    wp_list = []
    X = 3
    Y = 3
    for alt in np.arange(5, 25, 0.75):
        wp_list.append([X, Y, alt])
    for y in np.arange(3, 10, 1):
        wp_list.append([X, y, 25])
    Y = 10
    for x in np.arange(3, 10, 1):
        wp_list.append([x, Y, 25])
    X = 10
    for y in np.arange(Y, 3, -1):
        wp_list.append([X, y, 25])
    Y = 0
    for x in np.arange(X, 3, -1):
        wp_list.append([x, Y, 25])

    return wp_list

def get_distance(point):
    goalX, goalY, goalZ = point
    distance = math.sqrt((goalX-posX)**2 + (goalY-posY)**2 + (goalZ-posZ)**2)
    return distance

def start_mission(wps):
    setpoint_msg = PoseStamped()
    setpoint_msg.header.stamp = rospy.Time.now()
    setpoint_msg.header.frame_id = "map"

    setpoint_msg.pose.orientation.x = 0
    setpoint_msg.pose.orientation.y = 0
    setpoint_msg.pose.orientation.z = 0
    setpoint_msg.pose.orientation.w = 1

    for i, point in enumerate(wps):
        setpoint_msg.pose.position.x = point[0]
        setpoint_msg.pose.position.y = point[1]
        setpoint_msg.pose.position.z = point[2]
        goto_pub.publish(setpoint_msg)
        while(get_distance(point) > WAYPOINT_RADIUS):
            res = set_mode(0, "OFFBOARD")
            goto_pub.publish(setpoint_msg)
            rate.sleep()
        rospy.loginfo(f"Arrived at waypoint {i}/{len(wps)}")
    rospy.loginfo("Mission completed.")

def main():
    global rate
    rate = rospy.Rate(20)
    while(conn_status == False):
        rospy.loginfo("Waiting for connection...")
        rate.sleep()

    sample_msg = PoseStamped()
    sample_msg.header.stamp = rospy.Time.now()
    sample_msg.header.frame_id = "map"

    sample_msg.pose.orientation.x = 0
    sample_msg.pose.orientation.y = 0
    sample_msg.pose.orientation.z = 0
    sample_msg.pose.orientation.w = 1

    sample_msg.pose.position.x = 3
    sample_msg.pose.position.y = 3
    sample_msg.pose.position.z = 3

    while (not armed):
        arm_quad(True)
        rospy.loginfo("Trying to arm.")
        #rate.sleep()
        time.sleep(0.5)
    rospy.loginfo("Vehicle armed.")

    res1 = set_mode(0, "AUTO.TAKEOFF")
    rospy.loginfo("Mode set to AUTO.TAKEOFF")
    for i in range(100):
        goto_pub.publish(sample_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC} - {i}")
        rate.sleep()
        if (rospy.is_shutdown()):
            break
    
    arm_quad(True)
    rospy.loginfo("Vehicle armed")
    res2 = set_mode(0, "OFFBOARD")
    rospy.loginfo("Mode set to OFFBOARD")
    
    waypoints = generate_waypoints()
    start_mission(waypoints)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("waypoint_publisher2", anonymous=False)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/mavros/state", State, state_callback)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arm_quad = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    goto_pub = rospy.Publisher(PUB_TOPIC, PoseStamped, queue_size=5)
    time.sleep(2)
    main()