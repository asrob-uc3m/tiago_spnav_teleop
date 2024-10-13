#!/usr/bin/env python

import argparse
import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController

if __name__ == "__main__":
  rospy.init_node("stop_spnav")

  parser = argparse.ArgumentParser()
  parser.add_argument("--dual", type=str, default=None)
  args = parser.parse_args()

  if args.dual is not None:
    if args.dual != "left" and args.dual != "right":
      rospy.logfatal("Invalid argument for --dual: %s" % args.dual)
      exit(1)

    default_controllers = ["arm_%s_controller" % args.dual, "gripper_%s_controller" % args.dual]
  else:
    default_controllers = ["arm_controller", "gripper_controller"]

  rospy.wait_for_service('controller_manager/list_controllers')
  rospy.wait_for_service('controller_manager/switch_controller')

  manager_list = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
  manager_switch = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

  controllers = manager_list()

  start_controllers = default_controllers
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
