#!/usr/bin/env python
import sys
#from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import rospy
import moveit_msgs.msg
import moveit_commander
#import geometry_msgs.msg


class Commander:
    """ Move group planner
    """
    
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ovis_control", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        
        rospy.loginfo(self.group.get_planning_frame())

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self.target_pos = self.group.get_current_pose().pose

        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

        rospy.spin()


    # called when joy cmd_joy message is received
    def joy_callback(self, data):

        x = data.axes[0]
        y = data.axes[1]
        z = data.axes[4]

        scalling = 0.05

        deadzone = rospy.get_param("/joy_node/deadzone")

        #print(self.group.get_current_pose().pose.position)


        if abs(x) > deadzone or abs(y) > deadzone or abs(z) > deadzone:
            self.target_pos = self.group.get_current_pose().pose
            self.target_pos.position.x += x * scalling
            self.target_pos.position.y += y * scalling
            self.target_pos.position.z += z * scalling 

            # rospy.loginfo(self.group.get_current_joint_values())
            rospy.loginfo(self.target_pos)

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
