#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"

#include <sstream>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

const double pi = 3.1415;

const int ekf_frequency = 100;

const int max_markers = 1000;
int marker_id = 0;

ros::Time last_update_time;
bool first_update = true;

Matrix<float, 4, 1> current_state = Matrix<float, 4, 1>::Zero(); // x, y, theta, v
Matrix<float, 4, 4> current_covariance = Matrix<float, 4, 4>::Zero();

void init_ekf()
{
  current_covariance(0,0) = 1;
  current_covariance(1,1) = 1;
  current_covariance(2,2) = 1;
  current_covariance(3,3) = 1;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  double dt;
  if(first_update) {
    last_update_time = imu_msg->header.stamp;
    first_update = false;
    return;
  }
  else {
    dt = (imu_msg->header.stamp - last_update_time).toSec();
    last_update_time = imu_msg->header.stamp;
  }
  ROS_DEBUG("Imu data received");
  Matrix<float, 2, 2> Q = Matrix<float, 2, 2>::Zero();
  Q(0,0) = 0.5;
  Q(1,1) = 0.1;

  current_state(0) = current_state(0) + current_state(3)*cos(current_state(2))*dt;
  current_state(1) = current_state(1) + current_state(3)*sin(current_state(2))*dt;
  current_state(2) = current_state(2) + imu_msg->angular_velocity.z*dt;
  current_state(3) = current_state(3) + imu_msg->linear_acceleration.x*dt;

  Matrix<float, 4, 4> grad_f_x = Matrix<float,4,4>::Zero();

  grad_f_x(0,0) = 1;
  grad_f_x(0,2) = -current_state(3)*sin(current_state(2))*dt;
  grad_f_x(0,3) = cos(current_state(2))*dt;
  grad_f_x(1,1) = 1;
  grad_f_x(1,2) = current_state(3)*cos(current_state(2))*dt;
  grad_f_x(1,3) = sin(current_state(2))*dt;
  grad_f_x(2,2) = 1;
  grad_f_x(3,3) = 1;

  Matrix<float,4,2> grad_f_v = Matrix<float,4,2>::Zero();
  grad_f_v(2,0) = -dt;
  grad_f_v(3,1) = dt;
  current_covariance = grad_f_x*current_covariance*grad_f_x.transpose()+grad_f_v*Q*grad_f_v.transpose();

}

void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& gps_msg)
{
  Matrix<float, 2, 2> R = Matrix<float, 2, 2>::Zero();
  R(0,0) = 2;
  R(1,1) = 2;

  Matrix<float, 2, 1> h;
  h(0) = current_state(0);
  h(1) = current_state(1);

  Matrix<float, 2, 1> z;
  z(0) = gps_msg->x;
  z(1) = gps_msg->y;

  Matrix<float, 2, 4> grad_h_x = Matrix<float, 2, 4>::Zero();
  grad_h_x(0, 0) = 1;
  grad_h_x(1, 1) = 1;

  Matrix<float, 2, 2> grad_h_w = Matrix<float, 2, 2>::Zero();
  grad_h_w(0, 0) = 1;
  grad_h_w(1, 1) = 1;

  Matrix<float, 2, 2> S = grad_h_w*R*grad_h_w.transpose() + grad_h_x*current_covariance*grad_h_x.transpose();
  Matrix<float, 4, 2> W = current_covariance*grad_h_x.transpose()*S.inverse();

  current_state = current_state+(W*(z-h));
  current_covariance = current_covariance-W*S*W.transpose();

}

void speedometerCallback(const std_msgs::Float64::ConstPtr& speedometer_msg)
{
  Matrix<float, 1, 1> R;
  R(0,0) = 0.01;

  Matrix<float, 1, 1> h;
  h(0) = current_state(3);

  Matrix<float, 1, 1> z;
  z(0) = speedometer_msg->data;

  Matrix<float, 1, 4> grad_h_x = Matrix<float, 1, 4>::Zero();
  grad_h_x(0, 3) = 1;

  Matrix<float, 1, 1> grad_h_w;
  grad_h_w(0, 0) = 1;

  Matrix<float, 1, 1> S = grad_h_w*R*grad_h_w.transpose() + grad_h_x*current_covariance*grad_h_x.transpose();
  Matrix<float, 4, 1> W = current_covariance*grad_h_x.transpose()*S.inverse();

  current_state = current_state+(W*(z-h));
  current_covariance = current_covariance-W*S*W.transpose();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_node");

  ros::NodeHandle n;

  ros::Publisher estimate_pub = n.advertise<geometry_msgs::Pose2D>("/ekf_estimate", 1);
  ros::Publisher estimate_rviz_pub = n.advertise<visualization_msgs::Marker>("/ekf_estimate_rviz", 1);
  ros::Subscriber imu_sub = n.subscribe("/imu", 1, imuCallback);
  ros::Subscriber gps_sub = n.subscribe("/gps", 1, gpsCallback);
  ros::Subscriber speedometer_sub = n.subscribe("/speedometer", 1, speedometerCallback);


  ros::Rate loop_rate(ekf_frequency);

  init_ekf();

  while (ros::ok())
  {

    // filter update will be handled inside callbacks
    ros::spinOnce();
      ROS_INFO_STREAM(current_state);

    // publish the current estimate
    geometry_msgs::Pose2D estimate_msg;
    estimate_msg.x = current_state(0);
    estimate_msg.y = current_state(1);
    estimate_msg.theta = current_state(2);
    estimate_pub.publish(estimate_msg);

    // RViz visualization
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ekf_estimate";
    marker.id = marker_id++;
    marker_id = marker_id % max_markers;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = current_state(0);
    marker.pose.position.y = current_state(1);
    marker.pose.position.z = 0;
    tf::Quaternion q =  tf::createQuaternionFromRPY(0, 0, current_state(2));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 4;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();


    estimate_rviz_pub.publish(marker);

    loop_rate.sleep();
  }


  return 0;
}
