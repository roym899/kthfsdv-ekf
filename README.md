# kthfsdv-ekf
Extended Kalman filter implementation, fusing IMU, speedometer and GPS measurements

## How to run
```console
cd catkin_ws
catkin_make
roscore
rosrun kthfsdv-ekf ekf_node
rosrun kthfsdv-ekf data_publisher.py
rviz
```

In RViz, Add > By Topic > ekf_estimate_rviz > Marker to see a short term history of EKF estimates.
