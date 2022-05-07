#!/usr/bin/env python3
from re import X
from vine import transform
import rospy
import tf, tf.msg
import tf2_ros
import geometry_msgs.msg
import time
from tf.transformations import quaternion_from_euler

def state_cb(data):
    global posX
    global posY
    global posZ
    global rotX
    global rotY
    global rotZ
    global rotW
    
    posX = data.pose.position.x
    posY = data.pose.position.y
    posZ = data.pose.position.z

    rotX = data.pose.orientation.x
    rotY = data.pose.orientation.y
    rotZ = data.pose.orientation.z
    rotW = data.pose.orientation.w
    
    broadcast_base_link_tf()
    broadcast_camera_frame()


def broadcast_camera_frame():
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "/robot_camera_link"

    static_transformStamped.transform.translation.x = -0.158979
    static_transformStamped.transform.translation.y = -0.04405
    static_transformStamped.transform.translation.z = 0.045074

    #q = quaternion_from_euler(0, 1.57142857143, 0)
    q = quaternion_from_euler(0, 0, 1.57142857143*2)

    static_transformStamped.transform.rotation.x = q[0]
    static_transformStamped.transform.rotation.y = q[1]
    static_transformStamped.transform.rotation.z = q[2]
    static_transformStamped.transform.rotation.w = q[3]

    broadcaster_cam.sendTransform(static_transformStamped)

def broadcast_base_link_tf():
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "map"
    transformStamped.child_frame_id = "base_link"

    transformStamped.transform.translation.x = posX
    transformStamped.transform.translation.y = posY
    transformStamped.transform.translation.z = posZ

    transformStamped.transform.rotation.x = rotX
    transformStamped.transform.rotation.y = rotY
    transformStamped.transform.rotation.z = rotZ
    transformStamped.transform.rotation.w = rotW

    tfm = tf.msg.tfMessage([transformStamped])
    print("X: {0}, Y: {1}, Z: {2}".format(posX, posY, posZ))
    pub_tf.publish(tfm)

if __name__ == "__main__":
    global pub_tf 
    global transformStamped
    global broadcaster_cam
    global static_transformStamped

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    transformStamped = geometry_msgs.msg.TransformStamped()
    time.sleep(1)

    rospy.init_node("camera_frame_bc_node", anonymous=False)
    broadcaster_cam = tf2_ros.StaticTransformBroadcaster()
    pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=5)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, state_cb)

    rospy.spin()