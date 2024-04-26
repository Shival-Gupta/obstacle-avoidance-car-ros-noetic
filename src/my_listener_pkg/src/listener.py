#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def sensor_data_callback(msg):
    rospy.loginfo("Sensor distances: %s" % msg.data)

def listener():
    rospy.init_node('sensor_data_listener', anonymous=True)
    rospy.Subscriber('sensor_data', String, sensor_data_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
