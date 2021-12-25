#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmJointAnglesAction.h>

typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> Client;
float joints_degree[6];
float joystick_axes[3];
void setJointCommand()
{
  Client client("/c2s6s000_driver/joints_action/joint_angles", true);
  client.waitForServer();
  ROS_INFO("joint 1 = [%f] 2 = [%f] 3 = [%f] 4 = [%f] 5 = [%f] 6 = [%f] 7 = [%f]\n", joints_degree[0], joints_degree[1],
           joints_degree[2], joints_degree[3], joints_degree[4], joints_degree[5], joints_degree[6]);

  if (joints_degree[2] == 0.0)
  {
    printf("joints_degree[2] = [%f]\n", joints_degree[2]);
    return;
  }

  if (joystick_axes[1] < 0.05 && joystick_axes[1] > -0.05)
  {
    return;
  }

  float direction = joystick_axes[1] > 0.0 ? 1.0 : -1.0;

  kinova_msgs::ArmJointAnglesActionGoal goal;
  goal.goal.angles.joint1 = joints_degree[0];
  goal.goal.angles.joint2 = joints_degree[1];
  goal.goal.angles.joint3 = joints_degree[2] + direction * 3;
  goal.goal.angles.joint4 = joints_degree[3];
  goal.goal.angles.joint5 = joints_degree[4];
  goal.goal.angles.joint6 = joints_degree[5];
  goal.goal.angles.joint7 = joints_degree[6];

  client.sendGoal(goal.goal);
  if (client.waitForResult(ros::Duration(20.0)))
  {
    ROS_INFO("Goal acheived\n");
  }
  else
  {
    ROS_INFO("Goal failed\n");
    client.cancelAllGoals();
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("joy axes 0 = [%f] 1 = [%f] 2 = [%f] 3 = [%f]\n", joy->axes[0], joy->axes[1], joy->axes[3], joy->axes[4]);
  joystick_axes[0] = joy->axes[0];
  joystick_axes[1] = joy->axes[1];
  joystick_axes[2] = joy->axes[3];
  joystick_axes[3] = joy->axes[4];
}

void getJointCommand(const kinova_msgs::JointAnglesConstPtr& msg)
{
  // ROS_INFO("joint 1 = [%f] 2 = [%f] 3 = [%f] 4 = [%f] 5 = [%f] 6 = [%f] 7 = [%f]\n", msg->joint1, msg->joint2,
  //          msg->joint3, msg->joint4, msg->joint5, msg->joint6, msg->joint7);
  joints_degree[0] = msg->joint1;
  joints_degree[1] = msg->joint2;
  joints_degree[2] = msg->joint3;
  joints_degree[3] = msg->joint4;
  joints_degree[4] = msg->joint5;
  joints_degree[5] = msg->joint6;
  joints_degree[6] = msg->joint7;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ovis_demo_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/c2s6s000_driver/out/joint_command", 1000, getJointCommand);
  ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joyCallback);

  // ros::Rate loop_rate(10);
  while (ros::ok())
  {
    setJointCommand();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}