# Ovis

This repo contains the files required to interact with Capra's Ovis 6DOF arm.

## Dependencies

- [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq) which use `robotiq_2f_140_gripper_visualization`
- [clubcapra/jog_control](https://github.com/clubcapra/jog_control)
- [kinova-ros](https://github.com/Kinovarobotics/kinova-ros)

### How to setup the dependancies


In your workspace place git repository of the robotiq, kinova-ros and jogcontrol packages at the same level as the ovis package, so in the `src` folder. You can simply `git clone` the repository without issue. 

For the **kinova-ros** repository make sure you have the `melodic-devel` branch select. When cloning you can use this command to make sure that you have the right branch checked out:
```bash
git clone https://github.com/Kinovarobotics/kinova-ros.git -b melodic-devel
```

 Like so : 
```
src/
    jog_control/
    kinova-ros/
    ovis/
    robotiq/
```

### Launching the arm
    roslaunch ovis_bringup kinova_driver.launch

### Launch the simulation
    
    roslaunch ovis_gazebo gazebo.launch
    roslaunch ovis_moveit_config ovis_gazebo_demo.launch 
    roslaunch ovis_jog_launch ovis.launch 
