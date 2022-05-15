//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"

// void OvisArmJointVelocityCallback(const ovis_msgs::OvisArmJointVelocity::ConstPtr& msg)
// {
//   trajectory_point.Position.Actuators.Actuator1 = 0;
//   trajectory_point.Position.Actuators.Actuator2 = 0;
//   trajectory_point.Position.Actuators.Actuator3 = 0;
//   trajectory_point.Position.Actuators.Actuator4 = 0;
//   trajectory_point.Position.Actuators.Actuator5 = 0;
//   trajectory_point.Position.Actuators.Actuator6 = 0;
//   switch (msg->joint_index)
//   {
//     case 0:
//       trajectory_point.Position.Actuators.Actuator1 = msg->joint_velocity * number_of_degree_per_sec;
//       break;
//     case 1:
//       trajectory_point.Position.Actuators.Actuator2 = INVERSE * msg->joint_velocity * number_of_degree_per_sec;
//       break;
//     case 2:
//       trajectory_point.Position.Actuators.Actuator3 = msg->joint_velocity * number_of_degree_per_sec;
//       break;
//     case 3:
//       trajectory_point.Position.Actuators.Actuator4 = msg->joint_velocity * number_of_degree_per_sec;
//       break;
//     case 4:
//       trajectory_point.Position.Actuators.Actuator5 = msg->joint_velocity * number_of_degree_per_sec;
//       break;
//     case 5:
//       trajectory_point.Position.Actuators.Actuator6 = msg->joint_velocity * number_of_degree_per_sec;
//       break;
//   }
//   // sendBasicTrajectory(trajectory_point); TODO
// }

// bool HomePositionSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
// {
//   // sendBasicTrajectory(home_trajectory_point); TODO
//   res.message = "Send home position to actuators";
//   res.success = static_cast<unsigned char>(true);
//   return true;
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_arm_driver");
  ros::NodeHandle nh("~");
  boost::recursive_mutex api_mutex;

  bool is_first_init = true;
  std::string kinova_robotType = "";
  std::string kinova_robotName = "";

  // Retrieve the (non-option) argument:
  if ((argc <= 1) || (argv[argc - 1] == NULL))  // there is NO input...
  {
    std::cerr << "No kinova_robotType provided in the argument!" << std::endl;
    return -1;
  }
  else  // there is an input...
  {
    kinova_robotType = argv[argc - 1];
    ROS_INFO("kinova_robotType is %s.", kinova_robotType.c_str());
    if (!nh.getParam("robot_name", kinova_robotName))
    {
      kinova_robotName = kinova_robotType;
    }
    ROS_INFO("kinova_robotName is %s.", kinova_robotName.c_str());
  }

  while (ros::ok())
  {
    try
    {
      // ros::ServiceServer home_srv = nh.advertiseService("arm/home_joint_positions", HomePositionSrvCallback);
      // ros::Subscriber joint_goal_sub =
      //     nh.subscribe<ovis_msgs::OvisArmJointVelocity>("arm/joint_goal", 1, OvisArmJointVelocityCallback);
      kinova::KinovaComm comm(nh, api_mutex, is_first_init, kinova_robotType);
      kinova::KinovaArm kinova_arm(comm, nh, kinova_robotType, kinova_robotName);
      kinova::KinovaPoseActionServer pose_server(comm, nh, kinova_robotType, kinova_robotName);
      kinova::KinovaAnglesActionServer angles_server(comm, nh);
      kinova::KinovaFingersActionServer fingers_server(comm, nh);
      kinova::JointTrajectoryController joint_trajectory_controller(comm, nh);
      ros::spin();
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM(e.what());
      kinova::KinovaAPI api;
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      api.closeAPI();
      ros::Duration(1.0).sleep();
    }

    is_first_init = false;
  }
  return 0;
}
