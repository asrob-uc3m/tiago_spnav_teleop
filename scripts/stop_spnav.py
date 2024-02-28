#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController

if __name__ == "__main__":
  rospy.init_node("stop_spnav")

  rospy.wait_for_service('controller_manager/list_controllers')
  rospy.wait_for_service('controller_manager/switch_controller')

  manager_list = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
  manager_switch = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

  controllers = manager_list()

  start_controllers = ['arm_controller', 'gripper_controller']
  stop_controllers = []

  for controller in controllers.controller:
    if controller.name == "spnav_controller" and controller.state == "running":
      stop_controllers.append(controller.name)
      break

  rospy.loginfo("Switching controllers...")

  response = manager_switch(start_controllers=start_controllers, stop_controllers=stop_controllers, strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
