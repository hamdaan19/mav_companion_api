#!/usr/bin/env python3

from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Point, PoseStamped
from mav_companion_api.srv import TrajPlanner
from mavros_msgs.msg import State
import rospy
import time

def get_pose(data):
    global poseX
    global poseY
    global poseZ
    poseX = data.pose.position.x
    poseY = data.pose.position.y
    poseZ = data.pose.position.z

def state_callback(data):
    global conn_status 
    global current_mode
    global armed
    conn_status = data.connected
    current_mode = data.mode
    armed = data.armed

if __name__ == "__main__": 
    rospy.init_node("traj_msg_grabber", anonymous=False)
    rospy.wait_for_service('trajectory_planner')
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, get_pose)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)
    pub = rospy.Publisher("/trajectory", MultiDOFJointTrajectory, queue_size=5)

    rate = rospy.Rate(2)

    time.sleep(1)

    while(conn_status == False):
        rospy.loginfo("Waiting for connection...")
        rate.sleep()

    time.sleep(1)

    try: 
        traj_planner = rospy.ServiceProxy('trajectory_planner', TrajPlanner)
        resp = traj_planner(Point(poseX, poseY, poseZ), Point(poseX,poseY+5, 20))
        pub.publish(resp.trajectory)
        print(type(resp.trajectory))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
