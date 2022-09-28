#!/usr/bin/env python3

from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Point
from mav_companion_api.srv import TrajPlanner
import rospy

if __name__ == "__main__": 
    rospy.wait_for_service('trajectory_planner')
    try: 
        traj_planner = rospy.ServiceProxy('trajectory_planner', TrajPlanner)
        resp = traj_planner(Point(5, 4, 0), Point(5, 1, 10))
        print(type(resp))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
