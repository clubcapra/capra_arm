#include <algorithm>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

constexpr uint8_t NUMBER_OF_JOINTS = 6;
constexpr uint8_t NUMBER_OF_DEGREE_PER_GOAL = 5;

typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> Client;
static float joints_degree[6];
static float joystick_axes[3];
uint8_t joint_index;
static uint8_t joint_active[NUMBER_OF_JOINTS];
static std::string tmp_state_str;

void setJointCommand(Client& client)
{
  if (joints_degree[2] == 0.0)
  {
    ROS_INFO("joints_degree[2] = [%f]", joints_degree[2]);
    return;
  }

  // Act as a deadzone for the gamepad
  if (joystick_axes[1] < 0.1 && joystick_axes[1] > -0.1)
  {
    return;
  }

  std::fill_n(joint_active, NUMBER_OF_JOINTS, 0);
  switch (joint_index)
  {
    case 0:
      joint_active[0] = 1;
      break;
    case 1:
      joint_active[1] = 1;
      break;
    case 2:
      joint_active[2] = 1;
      break;
    case 3:
      joint_active[3] = 1;
      break;
    case 4:
      joint_active[4] = 1;
      break;
    case 5:
      joint_active[5] = 1;
      break;
  }

  int direction = (joystick_axes[1] > 0.0) ? 1 : -1;

  // ROS_INFO("Joystick axes [%f] Directory = [%d]  Joint_index = [%d] joint_active [%d] [%d] [%d] [%d] [%d] [%d] ",
  //          joystick_axes[1], direction, joint_index, joint_active[0], joint_active[1], joint_active[2],
  //          joint_active[3], joint_active[4], joint_active[5]);

  kinova_msgs::ArmJointAnglesActionGoal goal;
  goal.goal.angles.joint1 = joints_degree[0] + direction * joint_active[0] * NUMBER_OF_DEGREE_PER_GOAL;
  goal.goal.angles.joint2 = joints_degree[1] + direction * joint_active[1] * NUMBER_OF_DEGREE_PER_GOAL;
  goal.goal.angles.joint3 = joints_degree[2] + direction * joint_active[2] * NUMBER_OF_DEGREE_PER_GOAL;
  goal.goal.angles.joint4 = joints_degree[3] + direction * joint_active[3] * NUMBER_OF_DEGREE_PER_GOAL;
  goal.goal.angles.joint5 = joints_degree[4] + direction * joint_active[4] * NUMBER_OF_DEGREE_PER_GOAL;
  goal.goal.angles.joint6 = joints_degree[5] + direction * joint_active[5] * NUMBER_OF_DEGREE_PER_GOAL;
  goal.goal.angles.joint7 = joints_degree[6];

  ROS_INFO("GOAL = [%f][%f][%f][%f][%f][%f][%f]", goal.goal.angles.joint1, goal.goal.angles.joint2,
           goal.goal.angles.joint3, goal.goal.angles.joint4, goal.goal.angles.joint5, goal.goal.angles.joint6,
           goal.goal.angles.joint7);

  ROS_INFO("CANCELING PREVIOUS GOAL");
  if (client.getState() == actionlib::SimpleClientGoalState::ACTIVE)
  {
    client.cancelGoal();
    return;
  }

  ROS_INFO("SENDING NEW GOAL");
  client.sendGoal(goal.goal);
  // ros::Duration(1).sleep();
  // while (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  // {
  //   ROS_INFO("client state = [%s]", client.getState().toString().c_str());
  //   if (client.getState() == actionlib::SimpleClientGoalState::ABORTED)
  //   {
  //     client.cancelGoal();
  //   }
  // }

  // if (client.waitForResult(ros::Duration(2.0)))
  // {
  //   ROS_INFO("Goal acheived");
  // }
  // else
  // {
  //   ROS_INFO("Goal failed");
  //   client.cancelAllGoals();
  // }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // ROS_INFO("joy axes 0 = [%f] 1 = [%f] 2 = [%f] 3 = [%f]", joy->axes[0], joy->axes[1], joy->axes[3], joy->axes[4]);
  // ROS_INFO("joy buttons 0 = [%d] 1 = [%d] 2 = [%d] 3 = [%d] 4 = [%d] 5 = [%d] 6 = [%d] 7 = [%d] 8 = [%d] 9 = [%d] 10
  // = "
  //          "[%d]",
  //          joy->buttons[0], joy->buttons[1], joy->buttons[2], joy->buttons[3], joy->buttons[4], joy->buttons[5],
  //          joy->buttons[6], joy->buttons[7], joy->buttons[8], joy->buttons[9], joy->buttons[10]);
  joystick_axes[0] = joy->axes[0];
  joystick_axes[1] = joy->axes[1];
  joystick_axes[2] = joy->axes[3];
  joystick_axes[3] = joy->axes[4];

  // Left bumper = 4 Right bumper = 5
  if (joy->buttons[4] == 1 || joy->buttons[5] == 1)
  {
    ROS_INFO("joint_index = [%d]", joint_index);
    if ((joint_index == 0 && joy->buttons[4] == 1) || (joint_index == NUMBER_OF_JOINTS - 1 && joy->buttons[5] == 1))
    {
      return;
    }
    if (joy->buttons[4] == 1)
    {
      joint_index--;
      ROS_INFO("joint_index = [%d]", joint_index);
      return;
    }
    else if (joy->buttons[5] == 1)
    {
      joint_index++;
      ROS_INFO("joint_index = [%d]", joint_index);
      return;
    }
  }
}

void getJointCommand(const kinova_msgs::JointAnglesConstPtr& msg)
{
  // ROS_INFO("joint 1 = [%f] 2 = [%f] 3 = [%f] 4 = [%f] 5 = [%f] 6 = [%f] 7 = [%f]", msg->joint1, msg->joint2,
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
  joint_index = 0;

  ros::init(argc, argv, "ovis_demo_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/c2s6s000_driver/out/joint_command", 1000, getJointCommand);
  ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joyCallback);

  ros::AsyncSpinner spinner(0);
  spinner.start();

  // InitAPI();

  Client client("/c2s6s000_driver/joints_action/joint_angles", true);
  client.waitForServer();

  while (ros::ok())
  {
    setJointCommand(client);
  }
  ros::waitForShutdown();
  return 0;
}