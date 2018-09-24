#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <cmath>

const double pi = 3.1415;

const int ekf_frequency = 100;

double current_x = 0;
double current_y = 0;
double current_theta = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  ROS_DEBUG("Imu data received");
}

void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& gps_msg)
{
  ROS_DEBUG("GPS data received");
}

void speedometerCallback(const std_msgs::Float64::ConstPtr& speedometer_msg)
{
  ROS_DEBUG("Speedometer data received");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_node");

  ros::NodeHandle n;

  ros::Publisher estimate_pub = n.advertise<geometry_msgs::Pose2D>("/ekf_estimate", 1);
  ros::Subscriber imu_sub = n.subscribe("/imu", 1, imuCallback);
  ros::Subscriber gps_sub = n.subscribe("/gps", 1, gpsCallback);
  ros::Subscriber speedometer_sub = n.subscribe("/speedometer", 1, speedometerCallback);

  ros::Rate loop_rate(ekf_frequency);

  while (ros::ok())
  {
    // filter update will be handled inside callbacks
    ros::spinOnce();

    // publish the current estimate
    geometry_msgs::Pose2D estimate_msg;
    estimate_msg.x = current_x;
    estimate_msg.y = current_y;
    estimate_msg.theta = current_theta;
    estimate_pub.publish(estimate_msg);

    loop_rate.sleep();
  }


  return 0;
}
