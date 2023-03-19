#! /usr/bin/env python

import control_msgs.msg
import actionlib
import rospy
from ovis_msgs.msg import OvisJointPosition

class OvisAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryActionResult()

    def __init__(self, name):
        self._action_name = name
        self._pub = rospy.Publisher(
            "ovis/arm/in/ovis_cartesian_goal", OvisJointPosition, queue_size=10)
        self._as = actionlib.SimpleActionServer(
            self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        deg2rad = 180/3.14159265359
        kinova_offset = 180
        inversed_joint = [4, 5]
        joints_pos = goal.trajectory.points[-1].positions
        
        joints_angle_offset = [
            (pos * deg2rad) + kinova_offset if joint_num in inversed_joint 
             else kinova_offset - (pos * deg2rad)
              for joint_num, pos in enumerate(list(joints_pos))]

        # the data to be sent, initialise the array
        data_to_send = OvisJointPosition()
        
        # assign the array with the value you want to send
        data_to_send.joint_positions = joints_angle_offset
        self._pub.publish(data_to_send)

        r.sleep()

        if success:
            self._as.set_succeeded()
            rospy.loginfo('%s: Succeeded' % self._action_name)


if __name__ == '__main__':
    rospy.init_node('ovis_action_server')
    server = OvisAction(
        "/ovis/joint_trajectory_controller/follow_joint_trajectory")
    rospy.spin()
