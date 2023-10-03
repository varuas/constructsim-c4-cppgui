#include "robot_gui/robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include <iomanip>
#include <sstream>

RobotGUI::RobotGUI() {
  ros::NodeHandle nh;
  distance_ = 0.0;
  robot_info_sub = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      "/robot_info", 1, &RobotGUI::robotInfoCallback, this);
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 2,
                                              &RobotGUI::odomMsgCallback, this);
}

void RobotGUI::odomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_data = *msg;
}

void RobotGUI::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_data = *msg;
  ROS_INFO_STREAM("Robot Info Data: " << robot_info_data.data_field_01);
}

void RobotGUI::run() {
  cv::Mat frame = cv::Mat(800, 300, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // General Info Panel
    cvui::window(frame, 30, 20, 250, 145, "Info");
    cvui::text(frame, 30, 40, robot_info_data.data_field_01);
    cvui::text(frame, 30, 55, robot_info_data.data_field_02);
    cvui::text(frame, 30, 70, robot_info_data.data_field_03);
    cvui::text(frame, 30, 85, robot_info_data.data_field_04);
    cvui::text(frame, 30, 100, robot_info_data.data_field_05);
    cvui::text(frame, 30, 115, robot_info_data.data_field_06);
    cvui::text(frame, 30, 130, robot_info_data.data_field_07);
    cvui::text(frame, 30, 145, robot_info_data.data_field_08);

    // Tele-op buttons
    if (cvui::button(frame, 100, 180, " Forward ")) {
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
    }
    if (cvui::button(frame, 100, 210, "   Stop  ")) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }
    if (cvui::button(frame, 30, 210, " Left ")) {
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
    }
    if (cvui::button(frame, 195, 210, " Right ")) {
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
    }
    if (cvui::button(frame, 100, 240, "Backward")) {
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
    }
    twist_pub.publish(twist_msg);

    // // Linear Velocity Panel
    cvui::window(frame, 30, 280, 120, 40, "Linear velocity:");
    cvui::printf(frame, 40, 300, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);
    // Angular Velocity Panel
    cvui::window(frame, 160, 280, 120, 40, "Angular velocity:");
    cvui::printf(frame, 170, 300, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);

    // Show Robot Position from Odom Topic
    cvui::window(frame, 30, 330, 250, 40,
                 "Estimated robot position (odometery)");
    cvui::printf(frame, 40, 355, 0.4, 0xff0000, "X=%0.2f, Y=%0.2f, Z=%0.2f",
                 odom_data.pose.pose.position.x, odom_data.pose.pose.position.y,
                 odom_data.pose.pose.position.z);

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}
