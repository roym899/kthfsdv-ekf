#!/usr/bin/env python

import rospy
import numpy as np
import os
import scipy.io as sio
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

def data_publisher():
    data = sio.loadmat("./src/kthfsdv-ekf/matlab/data.mat")
    gps = {}
    gps['x'] = data['in_data'][0][0][0][0][0][0][0].ravel()
    gps['y'] = data['in_data'][0][0][0][0][0][0][1].ravel()
    gps['z'] = data['in_data'][0][0][0][0][0][0][2].ravel()
    gps['t'] = data['in_data'][0][0][0][0][0][3].ravel()
    speedometer = {}
    speedometer['speed'] = data['in_data'][0][0][1][0][0][1].ravel()
    speedometer['t'] = data['in_data'][0][0][1][0][0][0].ravel()
    imu = {}
    imu['ax'] = data['in_data'][0][0][2][0][0][1][0,:]
    imu['ay'] = data['in_data'][0][0][2][0][0][1][1,:]
    imu['az'] = data['in_data'][0][0][2][0][0][1][2,:]
    imu['alphax'] = data['in_data'][0][0][2][0][0][2][0,:]
    imu['alphay'] = data['in_data'][0][0][2][0][0][2][1,:]
    imu['alphaz'] = data['in_data'][0][0][2][0][0][2][2,:]
    imu['t'] = data['in_data'][0][0][2][0][0][0].ravel()

    gps_pub = rospy.Publisher('gps', Pose2D, queue_size=1)
    speedometer_pub = rospy.Publisher('speedometer', Float64, queue_size=1)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
    rospy.init_node('data_publisher', anonymous=True)
    rate = rospy.Rate(1000)

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        time = rospy.get_time()-start_time

        # handle GPS data
        gps_indices = np.argwhere(gps['t'] < time).ravel()
        if gps_indices.size is not 0:
            for gps_index in gps_indices:
                gps_msg = Pose2D(x=gps['x'][gps_index],
                                 y=gps['y'][gps_index])
                gps_pub.publish(gps_msg)
            for key in gps:
                gps[key] = np.delete(gps[key], gps_indices)

        # handle Speedometer data
        speedometer_indices = np.argwhere(speedometer['t'] < time).ravel()
        if speedometer_indices.size is not 0:
            for speedometer_index in speedometer_indices:
                speedometer_msg = Float64(data=speedometer['speed'][speedometer_index])
                speedometer_pub.publish(speedometer_msg)
            for key in speedometer:
                speedometer[key] = np.delete(speedometer[key], speedometer_indices)

        # handle IMU data
        imu_indices = np.argwhere(imu['t'] < time).ravel()
        if imu_indices.size is not 0:
            for imu_index in imu_indices:
                imu_msg = Imu(linear_acceleration=Vector3(x=imu['ax'][imu_index],
                                                          y=imu['ay'][imu_index],
                                                          z=imu['az'][imu_index]),
                              angular_velocity=Vector3(x=imu['alphax'][imu_index],
                                                       y=imu['alphay'][imu_index],
                                                       z=imu['alphaz'][imu_index]))
                imu_pub.publish(imu_msg)
            for key in imu:
                imu[key] = np.delete(imu[key], imu_indices)

        rate.sleep()

if __name__ == '__main__':
    try:
        data_publisher()
    except rospy.ROSInterruptException:
        pass
