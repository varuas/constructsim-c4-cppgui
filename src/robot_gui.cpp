#include "robot_gui/robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

RobotGUI::RobotGUI() {
  ros::NodeHandle nh;
  robot_info_sub = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      "/robot_info", 1, &RobotGUI::robotInfoCallback, this);
}

void RobotGUI::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_data = *msg;
  ROS_INFO_STREAM("Robot Info Data: " << robot_info_data.data_field_01);
}

void RobotGUI::run() {
  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 40, 20, 250, 145, "Info");

    cvui::text(frame, 40, 40, robot_info_data.data_field_01);
    cvui::text(frame, 40, 55, robot_info_data.data_field_02);
    cvui::text(frame, 40, 70, robot_info_data.data_field_03);
    cvui::text(frame, 40, 85, robot_info_data.data_field_04);
    cvui::text(frame, 40, 100, robot_info_data.data_field_05);
    cvui::text(frame, 40, 115, robot_info_data.data_field_06);
    cvui::text(frame, 40, 130, robot_info_data.data_field_07);
    cvui::text(frame, 40, 145, robot_info_data.data_field_08);

    // std::string robot_info_data = "Data Received" <<

    // Show how many times the button has been clicked inside the window.
    // cvui::printf(frame, 45, 45, 0.4, 0xff0000, "Data received: %s",
    //  robot_info_data.data_field_01.c_str());

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
