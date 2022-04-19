#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
import time

PUB_TOPIC = "/mavros/setpoint_raw/local"
STATE_TOPIC = "mavros/state"

GOTO_POINT = [7, 3, 4] # [X, Y, Z]

def state_callback(data):
    global conn_status 
    global current_mode
    global armed
    conn_status = data.connected
    current_mode = data.mode
    armed = data.armed

def main():
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

    setpoint_msg.position.x = GOTO_POINT[0]
    setpoint_msg.position.y = GOTO_POINT[1]
    setpoint_msg.position.z = GOTO_POINT[2]

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
    for i in range(100):
        pub.publish(setpoint_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC} - {i}")
        rate.sleep()
        if (rospy.is_shutdown()):
            break

    arm_quad(True)
    rospy.loginfo("Vehicle armed")
    res2 = set_mode(0, "OFFBOARD")
    rospy.loginfo("Mode set to OFFBOARD")

    while not rospy.is_shutdown():
        pub.publish(setpoint_msg)
        rospy.loginfo(f"Message published to topic: {PUB_TOPIC}")
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("companion_api", anonymous=False)
    pub = rospy.Publisher(PUB_TOPIC, PositionTarget, queue_size=5)
    state_sub = rospy.Subscriber(STATE_TOPIC, State, state_callback)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arm_quad = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    main()