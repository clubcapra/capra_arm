#!/usr/bin/env python
import sys
#from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import rospy
import moveit_msgs.msg
import moveit_commander
from tf.transformations import quaternion_from_euler as qfe, quaternion_multiply as qm, euler_from_quaternion as efq
import tf.transformations
import numpy as np

#import geometry_msgs.msg


class Commander:
    """ Move group planner
    """

    
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ovis_control", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group= moveit_commander.MoveGroupCommander("manipulator")
        
        rospy.loginfo(self.group.get_planning_frame())

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self.target_pos = self.group.get_current_pose().pose

        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.r = rospy.Rate(1)

        rospy.spin()


    # called when joy cmd_joy message is received
    def joy_callback(self, data):

        x = -data.axes[1]
        y = -data.axes[0]
        z = data.axes[4]

        print(self.group.get_current_pose().pose.orientation)


        #self.group.set_pose_reference_frame("ovis_link_6")
        #self.group.set_pose_reference_frame("ovis_link_6")

        #print('angle deuler correpondant',self.group.get_current_rpy())
        #print('\n End effector: ',self.group.get_end_effector_link())
        #print('\n KNown constaint',self.group.get_known_constraints())
        #print('\n Planning frame',self.group.get_planning_frame())
        #print('\n Ref frame: ',self.group.get_pose_reference_frame())

        #self.r.sleep()

        select = data.buttons[4]


        scalling = 0.05

        deadzone = rospy.get_param("/joy_node/deadzone")

        #print(self.group.get_current_pose().pose.position)


        if abs(x) > deadzone or abs(y) > deadzone or abs(z) > deadzone:
            
            #self.group.set_end_effector_link("ovis_link_tool0")

            self.target_pos = self.group.get_current_pose().pose
            R, P, Y = self.group.get_current_rpy()
            self.group.set_rpy_target((R+z*scalling,P,Y))

            self.group.set_planning_time(1)
            self.group.set_num_planning_attempts(2)

            self.plan1 = self.group.plan()
            self.group.execute(self.plan1, wait=False)

            
                
            if select == 11:
                self.target_pos.position.x += x * scalling
                self.target_pos.position.y += y * scalling
                self.target_pos.position.z += z * scalling 
            elif 1 == 4:

                qrot = qfe(0,-3.1415/8,0)
                qori = qrot
                qori[0]=self.target_pos.orientation.x
                qori[1]=self.target_pos.orientation.y
                qori[2]=self.target_pos.orientation.z
                qori[3]=self.target_pos.orientation.w


                #print('voici qrot ',qrot)
                #print('et son type',{type(qrot)})

                qnew = qm(qrot, qori)
                #print('voici qnew ',qnew)
                #qnew = tf.transformations.unit_vector(qnew)

                #print('voici qnew normalize',qnew)
                

                self.target_pos.orientation.x = qnew[0]
                self.target_pos.orientation.y = qnew[1]
                self.target_pos.orientation.z = qnew[2]
                self.target_pos.orientation.w = qnew[3]


                

                #self.target_pos.orientation.x += x * scalling
                #self.target_pos.orientation.y += y * scalling
                #self.target_pos.orientation.z += z * scalling 


            # rospy.loginfo(self.group.get_current_joint_values())
            #rospy.loginfo(self.target_pos)

            self.group.set_pose_target(self.target_pos)

            self.group.set_planning_time(0.1)
            self.group.set_num_planning_attempts(1)

            self.plan1 = self.group.plan()
            self.group.execute(self.plan1, wait=False)

            #self.group.stop()
            #self.group.clear_pose_targets()


if __name__ == '__main__':
    try:
        my_planner = Commander()

    except rospy.ROSInterruptException:
        pass

    moveit_commander.roscpp_shutdown()
