# Ovis
This repo contains the files required to interact with Capra's Ovis 6DOF arm.

## Dependencies
- [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq) which use `robotiq_2f_140_gripper_visualization`
- [clubcapra/jog_control](https://github.com/clubcapra/jog_control)
- [kinova-ros](https://github.com/Kinovarobotics/kinova-ros)

### How to setup the dependancies
In your workspace clone the git repository of the robotiq, kinova-ros and jogcontrol packages at the same level as the ovis package, so in the `src` folder. You can simply `git clone` the repository without issue.

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

## Installing the JACO SDK
To be able to communicate with the arm you first need to install the Kinova JACO SDK.

You can find the SDK in the Club Capra Teams files in the Logiciel channel under the name [Kinova Windows & Linux SDK](https://etsmtl365.sharepoint.com/:f:/s/CLUB-Capra-GTO365/Eii1z9mHcN9Dvc0AyOcN3OYBYrsurvhW-9wYIrKGoaCG8Q?e=gsdTq0) (The link requires you to be inside ClubCapra's Teams)

For Linux, the SDK is said to only support Ubuntu 16.04, **but it also work on Ubuntu 18.04**. This might become an issue if we decide to move to a newer version of Ubuntu/ROS.

To install the SDK please follow these steps :
1. Copy the `Kinova_sdk1.5.1.zip` from the Logiciel channel inside the folder `Kinova Windows & Linux SDK`.
2. Unzip it how ever you want.
   1. `unzip Kinova_sdk1.5.1.zip -d Kinova_sdk1.5.1`
3. Change directory to `Kinova_sdk1.5.1/Ubuntu/16_04/64\ bits`
   1. `cd Kinova_sdk1.5.1/Ubuntu/16_04/64\ bits`
4. Change the execution permission on the `installSDK64.sh` script
   1. `chmod +x installSDK64.sh`
5. Then execute the script with `sudo`
   1. `sudo ./installSDK64.sh`
6. Enter your password
7. Leave the setting by default and install the SDK

## Moving the arm with DevelopmentCenter
To verify that the installation was successful you can simply connect to the arm with a USB cable and turn on the power switch on the Kinova controller. Then launch the `DevelopmentCenter`.

1. Navigate to the location of the SDK installation and then into the `GUI` folder.
   1. By default its going to be `/opt/JACO-SDK`
   2. `cd /opt/JACO-SDK/GUI`
2. Then launch the `DevelopmentCenter` executable.
   1. `./DevelopmentCenter`
   2. WARNING, make sure you use the `DevelopmentCenter` and not the `DevelopmentCenter.sh`, since it was not tested.
3. You will be presented with this window :
    ![DevelopmentCenter_with_annotation](/doc/img/DevelopmentCenter_with_annotation.png)
    1. Make sure that the Ovis name is written at the top and that you have selected the `USB Enable`. This will make sure that the connection work properly.
    2. If you're having issue with this part, please check the Troubleshooting the SDK section.
4. Once inside the DevelopmentCenter window, select the `Virtual Joystick` under the TOOLS section.
   1. Then you'll be presented with this window :
      1. ![VirtualJoystick](/doc/img/VirtualJoystick.png)
5. Now try and more the 3rd actuator by pressing the `+` sign
6. Then the arm must of moved!

## Moving the arm with ROS
There's more information to come on this part.

For now, the step are as follow :
1. Launch the kinova-ros with Ovis bringup launch file
   1. `roslaunch ovis_bringup kinova_driver.launch`
2. Then run the kinova-demo to make the arm move
   1. This will more the 3rd actuator by 5 degree
   2. `rosrun kinova_demo joints_action_client.py -v -r c2s6s000 degree -- 0 0 5 0 0 0`

### Troubleshooting
1. If you're having issue connecting to the arm with the `DevelopmentCenter` or with `kinova-ros`, it might be because of two issue
   1. First, it might be because the USB autosuspend is enable
      1. Please follow this link for disabling it : https://askubuntu.com/questions/1140925/how-can-i-disable-usb-autosuspend-on-ubuntu-18-04/1161074#1161074
   2. Second, it might be that you need to launch the DevelopmentCenter with root right. This can be fix in two method
      1. Simply launch the `DevelopmentCenter` with `sudo`
      2. Change the udev that was install with the SDK. This method is prefered since it is permenant
         1. Inside your `/etc/udev` folder you'll find the file : `10-kinova-arm.rules`
         2. Edit the file with sudo and change the `MODE:="666"` to `MODE:="777"`
         3. This will allow anyone to have access to the arm
            1. TODO in the future it would be better to have the user added into a group instead of changing the permission in the udev.

### Launch the simulation
    roslaunch ovis_gazebo gazebo.launch
    roslaunch ovis_moveit_config ovis_gazebo_demo.launch
    roslaunch ovis_jog_launch ovis.launch
