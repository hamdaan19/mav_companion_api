#!/usr/bin/env python3
import rospy
from mav_companion_api.msg import text


if __name__ == "__main__":
    rospy.init_node("constant_talker", anonymous=False)
    pub = rospy.Publisher("constant_talker/text", text, queue_size=5)

    text_msg = text()
    text_msg.text = "Checking in..."

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        pub.publish(text_msg)
        rate.sleep()