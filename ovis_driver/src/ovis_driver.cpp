#include <algorithm>
#include <string>
#include <iostream>
#include <vector>

#include "ovis_driver.hpp"

#include "ovis_msgs/OvisJointAngles.h"
#include "ovis_msgs/OvisJointGoal.h"
#include "ovis_msgs/OvisIKGoal.h"
#include "ovis_msgs/HomeJoint.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

#include <inttypes.h>
#include <string>
#include <stdexcept>
#include <dlfcn.h>
#include <vector>

constexpr uint8_t NUMBER_OF_JOINTS = 6;
constexpr uint8_t NUMBER_OF_DEGREE_PER_GOAL = 1;

static uint8_t joint_index;
static uint8_t joint_active[NUMBER_OF_JOINTS];
static bool bumper_left_pressed = false;
static bool bumper_right_pressed = false;
static AngularPosition joints_angles;
static TrajectoryPoint trajectory_point;
static TrajectoryPoint home_trajectory_point;

static unsigned int const LEFT_BUMPER_INDEX = 4;
static unsigned int const RIGHT_BUMPER_INDEX = 5;
static unsigned int const INVERSE = -1;

void sendJointCommand(std::vector<float> const joy_axes)
{
  if (joy_axes[1] < 0.1 && joy_axes[1] > -0.1)
  {
    return;
  }

  int direction = (joy_axes[1] > 0.0) ? 1 : -1;

  int result = 0;
  result = getAngularPosition(joints_angles);
  if (result != NO_ERROR_KINOVA)
  {
    ROS_ERROR("Could not get the angular position. Result = [%d]", result);
  }

  switch (joint_index)
  {
    case 0:
      trajectory_point.Position.Actuators.Actuator1 += direction * NUMBER_OF_DEGREE_PER_GOAL;
      break;
    case 1:
      trajectory_point.Position.Actuators.Actuator2 += INVERSE * direction * NUMBER_OF_DEGREE_PER_GOAL;
      break;
    case 2:
      trajectory_point.Position.Actuators.Actuator3 += direction * NUMBER_OF_DEGREE_PER_GOAL;
      break;
    case 3:
      trajectory_point.Position.Actuators.Actuator4 += direction * NUMBER_OF_DEGREE_PER_GOAL;
      break;
    case 4:
      trajectory_point.Position.Actuators.Actuator5 += direction * NUMBER_OF_DEGREE_PER_GOAL;
      break;
    case 5:
      trajectory_point.Position.Actuators.Actuator6 += direction * NUMBER_OF_DEGREE_PER_GOAL;
      break;
  }
  sendBasicTrajectory(trajectory_point);
}

bool checkButtonPressed(int const button, bool& button_pressed_value)
{
  bool return_value = false;
  if (button == 1)
  {
    button_pressed_value = true;
  }
  else if (button_pressed_value == true)
  {
    return_value = true;
    button_pressed_value = false;
  }
  else
  {
    return_value = false;
  }
  return return_value;
}

void manageJointIndex(const std::vector<int> joy_buttons)
{
  // Left bumper = 4 Right bumper = 5
  if (checkButtonPressed(joy_buttons[LEFT_BUMPER_INDEX], bumper_left_pressed) == true)
  {
    if (joint_index > 0)
    {
      joint_index--;
      ROS_INFO("Decrease joint_index to [%d]", joint_index);
    }
  }

  if (checkButtonPressed(joy_buttons[RIGHT_BUMPER_INDEX], bumper_right_pressed))
  {
    if (joint_index < NUMBER_OF_JOINTS - 1)
    {
      joint_index++;
      ROS_INFO("Increase joint_index to [%d]", joint_index);
    }
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // acquire lock for arm moving
  manageJointIndex(joy->buttons);

  if (joy->buttons[0] == 1)  // TODO Add constant for A button
  {
    sendJointCommand(joy->axes);
  }
}

int loadLibraries()
{
  API_command_lib = dlopen("USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
  if (API_command_lib == NULL)
  {
    ROS_FATAL("%s", dlerror());
    return 0;
  }

  API_comm_lib = dlopen("USBCommLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
  if (API_comm_lib == NULL)
  {
    ROS_FATAL("%s", dlerror());
    return 0;
  }

  return 1;
}

void* initCommandLayerFunction(const char* name)
{
  void* function_pointer = dlsym(API_command_lib, name);
  assert(function_pointer != NULL);
  return function_pointer;
}

void InitAPIKinova()
{
  if (loadLibraries() == 0)
  {
    ros::shutdown();
  }
  else
  {
    ROS_INFO("loading libraries successful");
  }

  int result = NO_ERROR_KINOVA;
  std::string serial_number = "not_set";

  initAPI = (int (*)())initCommandLayerFunction("InitAPI");
  getAPIVersion = (int (*)(int[API_VERSION_COUNT]))initCommandLayerFunction("GetAPIVersion");
  getDevices = (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int&))initCommandLayerFunction("GetDevices");
  refresDevicesList = (int (*)())initCommandLayerFunction("RefresDevicesList");
  setActiveDevice = (int (*)(KinovaDevice))initCommandLayerFunction("SetActiveDevice");
  getGeneralInformations = (int (*)(GeneralInformations&))initCommandLayerFunction("GetGeneralInformations");
  setClientConfigurations = (int (*)(ClientConfigurations))initCommandLayerFunction("SetClientConfigurations");
  getQuickStatus = (int (*)(QuickStatus&))initCommandLayerFunction("GetQuickStatus");

  getAngularPosition = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularPosition");
  setAngularControl = (int (*)())initCommandLayerFunction("SetAngularControl");
  sendAdvanceTrajectory = (int (*)(TrajectoryPoint))initCommandLayerFunction("SendAdvanceTrajectory");
  sendBasicTrajectory = (int (*)(TrajectoryPoint))initCommandLayerFunction("SendBasicTrajectory");

  int api_version[API_VERSION_COUNT];
  result = getAPIVersion(api_version);
  if (result != NO_ERROR_KINOVA)
  {
    ROS_ERROR("Could not get the Kinova API version. Result = [%d]", result);
  }

  ROS_INFO_STREAM("Initializing Kinova USB API (header version: " << COMMAND_LAYER_VERSION
                                                                  << ", library version: " << api_version[0] << "."
                                                                  << api_version[1] << "." << api_version[2] << ")");

  result = initAPI();

  if (result != NO_ERROR_KINOVA)
  {
    ROS_ERROR("Could not initialize Kinova API. Result = [%d]", result);
  }

  result = refresDevicesList();

  if (result != NO_ERROR_KINOVA)
  {
    ROS_ERROR("Could not refresh the devices list. Result = [%d]", result);
  }

  result = NO_ERROR_KINOVA;
  int devices_count = getDevices(devices_list_, result);

  if (result != NO_ERROR_KINOVA)
  {
    ROS_ERROR("Could not get the devices list. Result = [%d]", result);
  }

  bool found_arm = false;
  for (int device_i = 0; device_i < devices_count; device_i++)
  {
    // If no device is specified, just use the first available device
    if (serial_number == "" || serial_number == "not_set" ||
        std::strcmp(serial_number.c_str(), devices_list_[device_i].SerialNumber) == 0)
    {
      result = setActiveDevice(devices_list_[device_i]);
      if (result != NO_ERROR_KINOVA)
      {
        ROS_ERROR("Could not set the active device. Result [%d]", result);
      }

      GeneralInformations general_info;
      result = getGeneralInformations(general_info);
      if (result != NO_ERROR_KINOVA)
      {
        ROS_ERROR("Could not get general information about the device. Result [%d]", result);
      }

      ClientConfigurations configuration;
      setClientConfigurations(configuration);
      strncpy(configuration.Model, "Custom", strlen("Custom") + 1);

      QuickStatus quick_status;
      getQuickStatus(quick_status);

      robot_type = quick_status.RobotType;

      ROS_INFO_STREAM("Found " << devices_count << " device(s), using device at index " << device_i << " (model: "
                               << configuration.Model << ", serial number: " << devices_list_[device_i].SerialNumber
                               << ", code version: " << general_info.CodeVersion
                               << ", code revision: " << general_info.CodeRevision << ")");

      found_arm = true;
      if (found_arm)
      {
        break;
      }
    }
  }

  if (!found_arm)
  {
    ROS_ERROR("Could not find the specified arm (serial: %s) among the %d attached devices", serial_number.c_str(),
              devices_count);
    ROS_ERROR("Could not find the specified arm");
  }
}

void OvisJointGoalCallback(const ovis_msgs::OvisJointGoal::ConstPtr& msg)
{
  switch (msg->joint_index)
  {
    case 0:
      trajectory_point.Position.Actuators.Actuator1 = msg->joint_angle;
      break;
    case 1:
      trajectory_point.Position.Actuators.Actuator2 = msg->joint_angle;
      break;
    case 2:
      trajectory_point.Position.Actuators.Actuator3 = msg->joint_angle;
      break;
    case 3:
      trajectory_point.Position.Actuators.Actuator4 = msg->joint_angle;
      break;
    case 4:
      trajectory_point.Position.Actuators.Actuator5 = msg->joint_angle;
      break;
    case 5:
      trajectory_point.Position.Actuators.Actuator6 = msg->joint_angle;
      break;
  }

  sendBasicTrajectory(trajectory_point);
}

bool HomeJointCallback(ovis_msgs::HomeJointRequest& request, ovis_msgs::HomeJointResponse& response)
{
  int result = sendBasicTrajectory(home_trajectory_point);
  if (result == NO_ERROR_KINOVA)
  {
    response.home_joint_positions.at(0) = home_trajectory_point.Position.Actuators.Actuator1;
    response.home_joint_positions.at(1) = home_trajectory_point.Position.Actuators.Actuator2;
    response.home_joint_positions.at(2) = home_trajectory_point.Position.Actuators.Actuator3;
    response.home_joint_positions.at(3) = home_trajectory_point.Position.Actuators.Actuator4;
    response.home_joint_positions.at(4) = home_trajectory_point.Position.Actuators.Actuator5;
    response.home_joint_positions.at(5) = home_trajectory_point.Position.Actuators.Actuator6;
    return true;
  }
  return false;
}

int main(int argc, char** argv)
{
  joint_index = 0;

  ros::init(argc, argv, "ovis_driver_node");
  ros::NodeHandle nh;
  try
  {
    InitAPIKinova();
  }
  catch (const std::exception& exception)
  {
    ROS_ERROR("Exception was raised when attempting to initialize kinova API.");
    ROS_ERROR("Reason: %s", exception.what());
  }

  int result = 0;
  result = getAngularPosition(joints_angles);
  trajectory_point.InitStruct();
  trajectory_point.Position.Type = ANGULAR_POSITION;
  trajectory_point.Position.Delay = 0.0;
  trajectory_point.Position.Actuators = joints_angles.Actuators;
  ovis_msgs::OvisJointAngles joint_angles_msg;
  joint_angles_msg.joint_angles.resize(6);

  // Set home position as start position
  home_trajectory_point.Position.Actuators = joints_angles.Actuators;

  ros::ServiceServer home_srv = nh.advertiseService("ovis/home_joint_positions", HomeJointCallback);

  ros::Publisher joints_pub = nh.advertise<ovis_msgs::OvisJointAngles>("ovis/joint_angles", 1);

  ros::Subscriber joint_goal_sub = nh.subscribe<ovis_msgs::OvisJointGoal>("ovis/joint_goal", 1, OvisJointGoalCallback);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);
  while (ros::ok())
  {
    result = getAngularPosition(joints_angles);
    if (result == NO_ERROR_KINOVA)
    {
      joint_angles_msg.joint_angles.at(0) = joints_angles.Actuators.Actuator1;
      joint_angles_msg.joint_angles.at(1) = joints_angles.Actuators.Actuator2;
      joint_angles_msg.joint_angles.at(2) = joints_angles.Actuators.Actuator3;
      joint_angles_msg.joint_angles.at(3) = joints_angles.Actuators.Actuator3;
      joint_angles_msg.joint_angles.at(4) = joints_angles.Actuators.Actuator4;
      joint_angles_msg.joint_angles.at(5) = joints_angles.Actuators.Actuator5;
      joints_pub.publish(joint_angles_msg);
    }
    ros::spinOnce();
  }

  return 0;
}