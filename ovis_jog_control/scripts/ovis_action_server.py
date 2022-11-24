#! /usr/bin/env python

import rospy

import actionlib

import control_msgs.msg

class OvisAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        print(goal.trajectory.points[-1].positons)
        r.sleep()
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
        #    self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('ovis_action_server')
    server = OvisAction("/ovis/joint_trajectory_controller/follow_joint_trajectory")
    rospy.spin()