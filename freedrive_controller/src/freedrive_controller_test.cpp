#include <ur_robot_driver/robot_state_helper.h>
#include <ros/ros.h>

using namespace freedrive_controller;

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "freedrive_controller");
  ros::NodeHandle nh;

  RobotStateHelper state_helper(nh);

  ros::spin();
  return 0;
}