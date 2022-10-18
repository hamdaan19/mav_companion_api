#!/usr/bin/env python3
from torch import per_tensor_symmetric
import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.srv import SetMode, CommandBool
from mav_companion_api.srv import TrajPlanner
import time

PUB_TOPIC = "/mavros/setpoint_raw/local"
STATE_TOPIC = "mavros/state"

GOTO_POINT = [0, 0, 10] # [X, Y, Z]

def state_callback(data):
    global conn_status 
    global current_mode
    global armed
    conn_status = data.connected
    current_mode = data.mode
    armed = data.armed

def get_pose(data):
    global poseX
    global poseY
    global poseZ
    poseX = data.pose.position.x
    poseY = data.pose.position.y
    poseZ = data.pose.position.z

def main(traj_msg):
    rate = rospy.Rate(20)
    time.sleep(2)

    while(conn_status == False):
        rospy.loginfo("Waiting for connection...")
        rate.sleep()

    setpoint_msg = PositionTarget()
    setpoint_msg.header.stamp = rospy.Time.now()
    setpoint_msg.header.frame_id = "map"

    setpoint_msg.coordinate_frame = 1
    setpoint_msg.type_mask = 0

    setpoint_msg.position.x = 0
    setpoint_msg.position.y = 0
    setpoint_msg.position.z = 0

    setpoint_msg.velocity.x = 0
    setpoint_msg.velocity.y = 0
    setpoint_msg.velocity.z = 0

    setpoint_msg.acceleration_or_force.x = 0
    setpoint_msg.acceleration_or_force.y = 0
    setpoint_msg.acceleration_or_force.z = 0

    setpoint_msg.yaw = 0
    setpoint_msg.yaw_rate = 0

    arm_quad(True)
    rospy.loginfo("Vehicle armed")
    res1 = set_mode(0, "AUTO.TAKEOFF")
    rospy.loginfo("Mode set to AUTO.TAKEOFF")
    for i in range(10):
        pub.publish(setpoint_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC} - {i}")
        rate.sleep()
        if (rospy.is_shutdown()):
            break

    arm_quad(True)
    rospy.loginfo("Vehicle armed")
    res2 = set_mode(0, "OFFBOARD")
    rospy.loginfo("Mode set to OFFBOARD")

    for msg in traj_msg.points:

        setpoint_msg_2 = PositionTarget()
        setpoint_msg_2.header.stamp = rospy.Time.now()
        setpoint_msg_2.header.frame_id = "map"

        setpoint_msg_2.coordinate_frame = 1
        setpoint_msg_2.type_mask = 0

        setpoint_msg_2.position.x = msg.transforms.translation.x
        setpoint_msg_2.position.y = msg.transforms.translation.y
        setpoint_msg_2.position.z = msg.transforms.translation.z

        setpoint_msg_2.velocity.x = msg.velocities.linear.x
        setpoint_msg_2.velocity.y = msg.velocities.linear.y
        setpoint_msg_2.velocity.z = msg.velocities.linear.z

        setpoint_msg_2.acceleration_or_force.x = msg.accelerations.linear.x
        setpoint_msg_2.acceleration_or_force.y = msg.accelerations.linear.y
        setpoint_msg_2.acceleration_or_force.z = msg.accelerations.linear.z


    while not rospy.is_shutdown():
        pub.publish(setpoint_msg)
        res2 = set_mode(0, "OFFBOARD")
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC}")
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("companion_api", anonymous=False)
    pub = rospy.Publisher(PUB_TOPIC, PositionTarget, queue_size=5)
    state_sub = rospy.Subscriber(STATE_TOPIC, State, state_callback)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, get_pose)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arm_quad = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    traj_planner_srv = rospy.ServiceProxy('trajectory_planner', TrajPlanner)
    resp = traj_planner_srv(Point(poseX, poseX, poseZ), Point(poseX, poseY, poseZ+10))
    main(resp)