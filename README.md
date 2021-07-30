# Ovis

This repo contains the files required to interact with Capra's Ovis 6DOF arm.

## Dependencies
- This package requires robotiq's `robotiq_2f_140_gripper_visualization` available at [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq).

### Launch the simulation
    
    roslaunch ovis_gazebo gazebo.launch
    roslaunch ovis_moveit_config ovis_gazebo_demo.launch 
    roslaunch ovis_bringup spacenav_cpp.launch

### Launch the real robot arm

**TBD**
