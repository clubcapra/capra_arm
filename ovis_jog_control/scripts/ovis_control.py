#!/usr/bin/env python
import sys
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import rospy
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tft
import numpy as np
import math



class Commander:
    """ Move group planner
    """

    
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ovis_control", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        #self.group : moveit_commander.move_group.MoveGroupCommander = moveit_commander.MoveGroupCommander("manipulator")
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        rospy.Subscriber("command_xyz", Float64MultiArray, self.xyz_callback, queue_size=1)
        rospy.Subscriber("command_rpy", Float64MultiArray, self.rpy_callback, queue_size=1)
        rospy.Subscriber("command_pose", Float64MultiArray, self.pose_callback, queue_size=1)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_z = 0
        self.cmd_roll = 0
        self.cmd_pitch = 0
        self.cmd_yaw = 0
        self.local_ref = 1
        self.world_ref = not self.local_ref

        self.end_effector_list = ["ovis_link_6", "ovis_end_effector"]

        self.pre_pose = self.group.get_current_pose().pose 
        self.target_pos = self.group.get_current_pose().pose 

        self.scalling = 0.05
        self.deadzone = 0.001
        #self.deadzone = rospy.get_param("/joy_node/deadzone")

        self.print_cmd()

        rospy.spin()


    # ************************* Callback Function ****************************************

    def xyz_callback(self, data):
        '''
            Called when cmd_xyz message is received
        '''

        self.set_cmd_from_float_array(data)

        self.print_cmd()

        self.update_pre_pos_with_current_pose()
        self.update_position_with_world()

        self.plan_and_execute()

    def rpy_callback(self, data):
        '''
            Called when cmd_rpy message is received
        '''
        self.set_cmd_from_float_array(data)

        self.print_cmd()

        self.update_pre_pos_with_current_pose()
        self.update_orientation_with_world()

        self.plan_and_execute()

    def pose_callback(self, data):
        """
            Compute_pose_from_joy_callback
        """

        self.set_cmd_from_float_array(data)

        if self.toggle_local_world_ref:
            self.local_ref = not self.local_ref
            self.world_ref = not self.world_ref

        if self.toggle_end_effector:
            self.update_ref()

        self.print_cmd()

        self.update_pre_pos_with_current_pose()
       
        if self.position_changed():
            if self.local_ref:
                self.update_position_with_local()
            if self.world_ref:
                self.update_position_with_world()
            
        if self.orientation_changed():  
            if self.local_ref:
                self.update_orientation_with_local()
            if self.world_ref:
                self.update_orientation_with_world()
            
        if self.position_changed() or self.orientation_changed():
            self.plan_and_execute()

    def joy_callback(self, data):
        ''' 
            Called when joy message is received
        '''
        
        self.set_cmd_from_joy(data)


        if self.toggle_local_world_ref:
            self.local_ref = not self.local_ref
            self.world_ref = not self.world_ref
        
        if self.toggle_end_effector:
            self.update_ref()

        self.print_cmd()

        self.update_pre_pos_with_current_pose()

        if self.position_changed():
            if self.local_ref:
                self.update_position_with_local()
            if self.world_ref:
                self.update_position_with_world()
            
        if self.orientation_changed():  
            if self.local_ref:
                self.update_orientation_with_local()
            if self.world_ref:
                self.update_orientation_with_world()
            
        if self.position_changed() or self.orientation_changed():
            self.plan_and_execute()

    # ************************* Member Function ****************************************

    def position_changed(self):
        '''
            Return True if new xyz_cmd are received in joy data else 0
        '''
        return abs(self.cmd_x) > self.deadzone or abs(self.cmd_y) > self.deadzone or abs(self.cmd_z) > self.deadzone

    def orientation_changed(self):
        '''
            Return True if new rpy_cmd are received in joy data else 0
        '''
        return abs(self.cmd_roll) > self.deadzone or abs(self.cmd_pitch) > self.deadzone or abs(self.cmd_yaw) > self.deadzone

    def update_pre_pos_with_current_pose(self):
        """
            Update pre_pose and create and array from his orientation for matrix manipulation
        """

        self.pre_pose = self.group.get_current_pose().pose 

        self.pre_pose_q = np.array(
            (
                self.pre_pose.orientation.x,
                self.pre_pose.orientation.y,
                self.pre_pose.orientation.z,
                self.pre_pose.orientation.w,
            )
        )

    def update_position_with_local(self):
        '''
            Translate the target with the end effector as ref 
        '''

        local_move = np.array((self.cmd_x, self.cmd_y, self.cmd_z, 1.0))
        
        xyz_move = np.dot(tft.quaternion_matrix(self.pre_pose_q), local_move)

        self.target_pos.position.x = self.pre_pose.position.x + xyz_move[0]
        self.target_pos.position.y = self.pre_pose.position.y + xyz_move[1]
        self.target_pos.position.z = self.pre_pose.position.z + xyz_move[2]


    def update_orientation_with_local(self):
        '''
            Roll  is around X   \n
            Pitch is around Y   \n
            Yaw   is around Z
        '''

        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.05

        if self.cmd_yaw >= self.deadzone:
            yaw = yaw + DTHETA * 2
        elif self.cmd_yaw <= self.deadzone: 
            yaw = yaw - DTHETA * 2
        
        if self.cmd_pitch >= self.deadzone:
            pitch = pitch + DTHETA * 2
        elif self.cmd_pitch <= -self.deadzone: 
            pitch = pitch - DTHETA * 2

        if self.cmd_roll >= self.deadzone:
            roll = roll + DTHETA * 2
        elif self.cmd_roll <= -self.deadzone: 
            roll = roll - DTHETA * 2

        diff_q = tft.quaternion_from_euler(roll, pitch, yaw)
        new_q = tft.quaternion_multiply(self.pre_pose_q, diff_q)

        self.target_pos.orientation.x = new_q[0]
        self.target_pos.orientation.y = new_q[1]
        self.target_pos.orientation.z = new_q[2]
        self.target_pos.orientation.w = new_q[3]

    def update_position_with_world(self):
        '''
            Translate the target with the base link as ref 
        '''

        self.target_pos.position.x = self.pre_pose.position.x + self.cmd_x 
        self.target_pos.position.y = self.pre_pose.position.y + self.cmd_y
        self.target_pos.position.z = self.pre_pose.position.z + self.cmd_z 

    def update_orientation_with_world(self):
        '''
            Roll  is around X   \n
            Pitch is around Y  \n
            Yaw   is around Z
        '''
        
        r, p, y = self.group.get_current_rpy()

        r += self.cmd_roll
        p += self.cmd_pitch
        y += self.cmd_yaw

        quat = tft.quaternion_from_euler(r,p,y)

        self.target_pos.orientation.x = quat[0]
        self.target_pos.orientation.y = quat[1]
        self.target_pos.orientation.z = quat[2]
        self.target_pos.orientation.w = quat[3]

    def plan_and_execute(self):
        """ Set pose target, plan and execute
        """

        self.group.set_pose_target(self.target_pos)

        # self.group.go(wait=True)
        self.plan1 = self.group.plan()
        self.group.execute(self.plan1, wait=True)

        self.group.stop()
        self.group.clear_pose_targets()

    def update_ref(self):
        """ Change the end effector link
        """
        next_effector = self.end_effector_list.index(self.group.get_end_effector_link()) - 1
        self.group.set_end_effector_link(self.end_effector_list[next_effector])
    
    def set_cmd_from_float_array(self, data):
        """ Set cmd from float array send by rostopic pub or rqt
        """

        self.cmd_x = data.data[0] if len(data.data) >= 1 else 0
        self.cmd_y = data.data[1] if len(data.data) >= 2 else 0
        self.cmd_z = data.data[2] if len(data.data) >= 3 else 0
        self.cmd_roll = data.data[3] if len(data.data) >= 4 else 0
        self.cmd_pitch = data.data[4] if len(data.data) >= 5 else 0
        self.cmd_yaw = data.data[5] if len(data.data) >= 6 else 0
        self.toggle_local_world_ref = data.data[6] if len(data.data) >= 7 else 0
        self.toggle_end_effector = data.data[7] if len(data.data) >= 8 else 0

        self.cmd_x *= self.scalling
        self.cmd_y *= self.scalling
        self.cmd_z *= self.scalling
        self.cmd_roll *= self.scalling
        self.cmd_pitch *= self.scalling
        self.cmd_yaw *= self.scalling 

    
    def set_cmd_from_joy(self, data):
        """ Set cmd from joy msg
        """

        self.position_mode = True if data.axes[2] == -1 else False
        self.orientation_mode = True if data.axes[5] == -1 else False

        if self.position_mode and self.orientation_mode:
            self.position_mode = False
            self.orientation_mode = False

        if self.position_mode:
            self.cmd_x = data.axes[1]    # Left/Right Axis stick left
            self.cmd_y = -data.axes[0]   # Up/Down Axis stick left
            self.cmd_z = data.axes[4]    # Up/Down Axis stick right
            self.cmd_roll = 0
            self.cmd_pitch = 0
            self.cmd_yaw = 0

        if self.orientation_mode:
            self.cmd_x = 0
            self.cmd_y = 0
            self.cmd_z = 0
            self.cmd_yaw = data.axes[1]    # Left/Right Axis stick left
            self.cmd_roll = data.axes[0]   # Up/Down Axis stick left
            self.cmd_pitch = data.axes[4]  # Up/Down Axis stick right

        self.toggle_local_world_ref = data.buttons[7] # Select
        self.toggle_end_effector = data.buttons[3] # Y

    def print_cmd(self):
        """ Print useful informations (x, y, z, roll, pitch, yaw, ref)
        """

        print("\n*******************************************")
        print("x: {0:4}, Roll: {1:4}".format(self.cmd_x,self.cmd_roll))
        print("y: {0:4}, pitch:{1:4}".format(self.cmd_y,self.cmd_pitch))
        print("z: {0:4}, Yaw:  {1:4}".format(self.cmd_z,self.cmd_yaw))
        print("Reference: {0}".format("Local" if self.local_ref else "World"))
        print("End effector: {0}".format(self.group.get_end_effector_link()))
        print("Pose Ref Frame: {0}".format(self.group.get_pose_reference_frame()))
        print("*******************************************\n")


if __name__ == '__main__':
    try:
        my_planner = Commander()

    except rospy.ROSInterruptException:
        pass

    moveit_commander.roscpp_shutdown()
