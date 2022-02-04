#include <algorithm>
#include <string>
#include <iostream>
#include <vector>

#include "ovis_demo.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

#include <inttypes.h>
#include <string>
#include <stdexcept>
#include <dlfcn.h>

constexpr uint8_t NUMBER_OF_JOINTS = 6;
constexpr uint8_t NUMBER_OF_DEGREE_PER_GOAL = 3;

static float joints_degree[NUMBER_OF_JOINTS];

static uint8_t joint_index;
static uint8_t joint_active[NUMBER_OF_JOINTS];
static bool bumper_left_pressed = false;
static bool bumper_right_pressed = false;
static AngularPosition joints_angles;

void setJointDegree(float &actuator, float degree)
{
  actuator += degree;
}

void sendJointCommand(std::vector<float> const joy_axes)
{
  // Act as a deadzone for the gamepad
  // TODO remove and use launch parameter
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

  result = setAngularControl();
  if (result != NO_ERROR_KINOVA)
  {
    ROS_INFO("Could not set angular control. Result = [%d]", result);
  }

  TrajectoryPoint trajectory_point;
  trajectory_point.InitStruct();
  memset(&trajectory_point, 0, sizeof(trajectory_point));

  switch (joint_index)
  {
    case 0:
      setJointDegree(joints_angles.Actuators.Actuator1, direction * NUMBER_OF_DEGREE_PER_GOAL);
      break;
    case 1:
      setJointDegree(joints_angles.Actuators.Actuator2, direction * NUMBER_OF_DEGREE_PER_GOAL);
      break;
    case 2:
      setJointDegree(joints_angles.Actuators.Actuator3, direction * NUMBER_OF_DEGREE_PER_GOAL);
      break;
    case 3:
      setJointDegree(joints_angles.Actuators.Actuator4, direction * NUMBER_OF_DEGREE_PER_GOAL);
      break;
    case 4:
      setJointDegree(joints_angles.Actuators.Actuator5, direction * NUMBER_OF_DEGREE_PER_GOAL);
      break;
    case 5:
      setJointDegree(joints_angles.Actuators.Actuator6, direction * NUMBER_OF_DEGREE_PER_GOAL);
      break;
  }

  trajectory_point.Position.Delay = 0.0;
  trajectory_point.Position.Type = ANGULAR_POSITION;
  trajectory_point.Position.Actuators = joints_angles.Actuators;
  trajectory_point.LimitationsActive = 0;

  ROS_INFO("Joint[%d] degree = [%f]",1,joints_angles.Actuators.Actuator1);
  ROS_INFO("Joint[%d] degree = [%f]",2,joints_angles.Actuators.Actuator2);
  ROS_INFO("Joint[%d] degree = [%f]",3,joints_angles.Actuators.Actuator3);
  ROS_INFO("Joint[%d] degree = [%f]",4,joints_angles.Actuators.Actuator4);
  ROS_INFO("Joint[%d] degree = [%f]",5,joints_angles.Actuators.Actuator5);
  ROS_INFO("Joint[%d] degree = [%f]",6,joints_angles.Actuators.Actuator6);
  ROS_INFO("-----------------------------");
  ROS_INFO("Trajectory Joint[%d] degree = [%f]",1,trajectory_point.Position.Actuators.Actuator1);
  ROS_INFO("Trajectory Joint[%d] degree = [%f]",2,trajectory_point.Position.Actuators.Actuator2);
  ROS_INFO("Trajectory Joint[%d] degree = [%f]",3,trajectory_point.Position.Actuators.Actuator3);
  ROS_INFO("Trajectory Joint[%d] degree = [%f]",4,trajectory_point.Position.Actuators.Actuator4);
  ROS_INFO("Trajectory Joint[%d] degree = [%f]",5,trajectory_point.Position.Actuators.Actuator5);
  ROS_INFO("Trajectory Joint[%d] degree = [%f]",6,trajectory_point.Position.Actuators.Actuator6);
  ROS_INFO("-----------------------------");

  sendAdvanceTrajectory(trajectory_point);
}

bool checkButtonPressed(int const button, bool & button_pressed_value)
{
  bool return_value = false;
  if(button == 1)
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
  if(checkButtonPressed(joy_buttons[4], bumper_left_pressed) == true)
  {
    if(joint_index > 0)
    {
      joint_index--;
      ROS_INFO("Decrease joint_index to [%d]",joint_index);
    }
  }

  if(checkButtonPressed(joy_buttons[5], bumper_right_pressed)) 
  {
    if(joint_index < NUMBER_OF_JOINTS - 1)
    {
      joint_index++;
      ROS_INFO("Increase joint_index to [%d]",joint_index);
    }
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  manageJointIndex(joy->buttons);

  if(joy->buttons[0] == 1)
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
      break;
    }
  }

  if (!found_arm)
  {
    ROS_ERROR("Could not find the specified arm (serial: %s) among the %d attached devices", serial_number.c_str(),
              devices_count);
    ROS_ERROR("Could not find the specified arm");
  }
}

int main(int argc, char** argv)
{
  joint_index = 0;

  ros::init(argc, argv, "ovis_demo_node");
  ros::NodeHandle nh;
  ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joyCallback);

  try
  {
    InitAPIKinova();
  }
  catch (const std::exception& exception)
  {
    ROS_ERROR("Exception was raised when attempting to initialize kinova API.");
    ROS_ERROR("Reason: %s", exception.what());
  }

  ros::spin();
  return 0;
}