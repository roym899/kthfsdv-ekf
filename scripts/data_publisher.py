#!/usr/bin/env python

import rospy
import numpy as np

def data_publisher():
    # gps_pub = rospy.Publisher('gps', String, queue_size=1)
    # imu_pub = rospy.Publisher('imu', String, queue_size=1)
    # speedometer_pub = rospy.Publisher('speedometer', String, queue_size=1)
    rospy.init_node('data_publisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("test")
        rate.sleep()

if __name__ == '__main__':
    try:
        data_publisher()
    except rospy.ROSInterruptException:
        pass
