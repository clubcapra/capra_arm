#include "ovis_arm.hpp"

#include "ovis_msgs/OvisJointGoal.h"
#include "ovis_msgs/OvisIKGoal.h"
#include "ovis_msgs/HomeJoint.h"

#include <ros/ros.h>

#include <dlfcn.h>

static int number_of_degree_per_sec = 0;

static AngularPosition joints_angles;

static TrajectoryPoint trajectory_point;
static TrajectoryPoint home_trajectory_point;

static int const INVERSE = -1;

void printDebugInfo()
{
  ROS_INFO(" COMMAND SENT ON JOINT 1: [%f] \n", trajectory_point.Position.Actuators.Actuator1);
  ROS_INFO(" COMMAND SENT ON JOINT 2: [%f] \n", trajectory_point.Position.Actuators.Actuator2);
  ROS_INFO(" COMMAND SENT ON JOINT 3: [%f] \n", trajectory_point.Position.Actuators.Actuator3);
  ROS_INFO(" COMMAND SENT ON JOINT 4: [%f] \n", trajectory_point.Position.Actuators.Actuator4);
  ROS_INFO(" COMMAND SENT ON JOINT 5: [%f] \n", trajectory_point.Position.Actuators.Actuator5);
  ROS_INFO(" COMMAND SENT ON JOINT 6: [%f] \n", trajectory_point.Position.Actuators.Actuator6);
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
  trajectory_point.Position.Actuators.Actuator1 = 0;
  trajectory_point.Position.Actuators.Actuator2 = 0;
  trajectory_point.Position.Actuators.Actuator3 = 0;
  trajectory_point.Position.Actuators.Actuator4 = 0;
  trajectory_point.Position.Actuators.Actuator5 = 0;
  trajectory_point.Position.Actuators.Actuator6 = 0;
  switch (msg->joint_index)
  {
    case 0:
      trajectory_point.Position.Actuators.Actuator1 = msg->joint_velocity * number_of_degree_per_sec;
      break;
    case 1:
      trajectory_point.Position.Actuators.Actuator2 = INVERSE * msg->joint_velocity * number_of_degree_per_sec;
      break;
    case 2:
      trajectory_point.Position.Actuators.Actuator3 = msg->joint_velocity * number_of_degree_per_sec;
      break;
    case 3:
      trajectory_point.Position.Actuators.Actuator4 = msg->joint_velocity * number_of_degree_per_sec;
      break;
    case 4:
      trajectory_point.Position.Actuators.Actuator5 = msg->joint_velocity * number_of_degree_per_sec;
      break;
    case 5:
      trajectory_point.Position.Actuators.Actuator6 = msg->joint_velocity * number_of_degree_per_sec;
      break;
  }
  sendBasicTrajectory(trajectory_point);
}

bool HomeJointCallback(ovis_msgs::HomeJointRequest& request, ovis_msgs::HomeJointResponse& response)
{
  // TODO When call put back the arm in it's resting position & add button for it ????
  response.home_joint_positions.resize(6);
  response.home_joint_positions.at(0) = 111.4451;
  response.home_joint_positions.at(1) = 10.5468;
  response.home_joint_positions.at(2) = 89.9120;
  response.home_joint_positions.at(3) = 153.5546;
  response.home_joint_positions.at(4) = 176.5507;
  response.home_joint_positions.at(5) = 121.2007;

  response.current_joint_positions.resize(6);
  response.current_joint_positions.at(0) = home_trajectory_point.Position.Actuators.Actuator1;
  response.current_joint_positions.at(1) = home_trajectory_point.Position.Actuators.Actuator2;
  response.current_joint_positions.at(2) = home_trajectory_point.Position.Actuators.Actuator3;
  response.current_joint_positions.at(3) = home_trajectory_point.Position.Actuators.Actuator4;
  response.current_joint_positions.at(4) = home_trajectory_point.Position.Actuators.Actuator5;
  response.current_joint_positions.at(5) = home_trajectory_point.Position.Actuators.Actuator6;
  return true;

  // TODO Block until the joint are at the right position
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm");
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

  getAngularPosition(joints_angles);
  if (nh.getParam("/ovis/arm/number_of_degree_per_sec", number_of_degree_per_sec) == false)
  {
    ROS_ERROR("Missing number_of_degree_per_sec parameter");
  }

  trajectory_point.InitStruct();
  trajectory_point.Position.Type = ANGULAR_VELOCITY;
  trajectory_point.Position.Delay = 0.0;
  trajectory_point.Position.Actuators = joints_angles.Actuators;

  ros::ServiceServer home_srv = nh.advertiseService("arm/home_joint_positions", HomeJointCallback);
  ros::Subscriber joint_goal_sub = nh.subscribe<ovis_msgs::OvisJointGoal>("arm/joint_goal", 1, OvisJointGoalCallback);
  ros::spin();

  return 0;
}
