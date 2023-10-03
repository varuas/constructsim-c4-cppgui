#pragma once

#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "std_srvs/Trigger.h"

class RobotGUI {
public:
  RobotGUI();
  void run();

private:
  ros::Subscriber robot_info_sub;
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist_msg;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;
  robotinfo_msgs::RobotInfo10Fields robot_info_data;
  const std::string WINDOW_NAME = "Robot Teleop GUI";
  ros::Subscriber odom_sub;
  nav_msgs::Odometry odom_data;
  ros::ServiceServer get_distance_service_;
  double distance;
  void odomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
};