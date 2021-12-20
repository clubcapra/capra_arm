#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("joy axes 0 = [%f] 1 = [%f] 2 = [%f] 3 = [%f]\n", joy->axes[0], joy->axes[1], joy->axes[3], joy->axes[4]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ovis_demo_node");
  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joyCallback);
  ROS_INFO("Listening to /joy\n");
  ros::spin();

  return 0;
}