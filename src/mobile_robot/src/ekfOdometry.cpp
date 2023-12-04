#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include "sensor_msgs/JointState.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

class OdometryCalculator {
public:
  OdometryCalculator() : nh("~"){
    wheelRadius = 0.135;
    wheelDist   = 0.650;

    left_wheel_vel_ = 0.0;
    right_wheel_vel_ = 0.0;


    imu_yaw_ = 0.0;

    odometrySub = nh.subscribe("/mobile_robot/joint_states", 1, &OdometryCalculator::odometryCallBack, this);
    imu_sub_ = nh.subscribe("/imu", 1, &OdometryCalculator::imuCallBack, this);
    odometryPub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    last_time_ = ros::Time::now();
  }

  void odometryCallBack(const sensor_msgs::JointState::ConstPtr& data){

    left_wheel_vel_  = data->velocity[0];
    right_wheel_vel_ = data->velocity[1];

    // Odometry Hesaplama
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();

    double linear_vel   = ((left_wheel_vel_ * wheelRadius) + (right_wheel_vel_ * wheelRadius)) / 2;
    double angular_vel  = ((right_wheel_vel_  * wheelRadius) - (left_wheel_vel_ * wheelRadius)) / wheelDist;

    double delta_x = linear_vel * cos(theta_) * dt;
    double delta_y = linear_vel * sin(theta_) * dt;
    double delta_theta = angular_vel * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // IMU yaw değerini ata.
    theta_ = imu_yaw_;

    // Odometry mesajını yayınlama.
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;

    odometryPub_.publish(odom_msg);

    ROS_INFO("%lf\t%lf\t%lf",x_, y_, theta_);

    last_time_ = current_time;
  }

  void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
    // Get yaw angle from IMU
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, imu_yaw_);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber odometrySub;
  ros::Subscriber imu_sub_;
  ros::Publisher odometryPub_;

  double left_wheel_vel_;
  double right_wheel_vel_;
  double wheelRadius;
  double wheelDist;

  double x_;
  double y_;
  double theta_;

  double imu_yaw_;
  double roll_;
  double pitch_;

  ros::Time last_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_calculator");

  OdometryCalculator odometry_calculator;

  ros::spin();

  return 0;
}
