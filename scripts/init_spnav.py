#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController

if __name__ == "__main__":
  rospy.init_node("init_spnav")
  rospy.loginfo("Waiting for play_motion...")
  client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("joint_states", JointState)
  rospy.sleep(3.0)

  rospy.loginfo("Extend arm...")
  goal = PlayMotionGoal()
  goal.motion_name = 'extend'
  goal.skip_planning = False

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("Arm extended.")

  rospy.wait_for_service('controller_manager/switch_controller')
  manager = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
  rospy.loginfo("Switching controllers...")
  response = manager(start_controllers=['spnav_controller'], stop_controllers=['arm_controller', 'gripper_controller'], strictness=2)
  if not response.ok:
    rospy.logfatal("Failed to switch controllers")
  rospy.loginfo("...done.")
