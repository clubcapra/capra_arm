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
        rospy.loginfo("new_position = %f", self.group.get_planning_frame())

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self.pose = self.group.get_current_pose().pose

        rospy.loginfo(20*"*")
        rospy.loginfo(self.pose)
        rospy.loginfo(20*"*")

        rospy.loginfo(self.group.get_current_joint_values())

        self.target_pos = self.group.get_current_pose().pose

        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

        rospy.spin()

    # called when joy cmd_joy message is received
    def joy_callback(self, data):

        scalling = 0.01

        self.target_pos.position.x = self.group.get_current_pose().pose.position.x + \
            data.axes[0] * scalling
        self.target_pos.position.y = self.group.get_current_pose().pose.position.y + \
            data.axes[1] * scalling
        self.target_pos.position.z = self.group.get_current_pose().pose.position.z + \
            data.axes[3] * scalling

        # rospy.loginfo(self.group.get_current_joint_values())
        rospy.loginfo(self.target_pos)

        self.group.set_pose_target(self.target_pos)
        self.group.set_planning_time(5)
        self.group.set_num_planning_attempts(1)
        self.plan1 = self.group.plan()
        rospy.loginfo("Return result of go: {}", self.group.go(wait=True))
        self.group.execute(self.plan1, wait=True)

        self.group.stop()
        self.group.clear_pose_targets()


if __name__ == '__main__':
    try:
        my_planner = Commander()

    except rospy.ROSInterruptException:
        pass

    moveit_commander.roscpp_shutdown()
