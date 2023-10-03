#pragma once

#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_msgs/Float64.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
public:
  RobotGUI();
  void run();

private:
  ros::Subscriber robot_info_sub;
  robotinfo_msgs::RobotInfo10Fields robot_info_data;

  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

  const std::string WINDOW_NAME = "Robot Teleop GUI";
};