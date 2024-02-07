#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import SwitchController


if __name__ == "__main__":
  rospy.init_node("stop_spnav")

  rospy.wait_for_service('controller_manager/switch_controller')
  manager = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
  rospy.loginfo("Switching controllers...")
  response = manager(start_controllers=['arm_controller', 'gripper_controller'], stop_controllers=['spnav_controller'], strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
