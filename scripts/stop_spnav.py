#!/usr/bin/env python

import argparse
import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController

if __name__ == "__main__":
  rospy.init_node("stop_spnav")

  parser = argparse.ArgumentParser()
  parser.add_argument("--arm", type=str, default=None)
  args = parser.parse_args()

  if args.arm is not None: # TIAGo++
    if args.arm not in ["left", "right", "both"]:
      rospy.logfatal("Invalid argument for --arm: %s" % args.arm)
      exit(1)

    if args.arm == "left":
      default_controllers = ["arm_left_controller", "gripper_left_controller"]
    elif args.arm == "right":
      default_controllers = ["arm_right_controller", "gripper_right_controller"]
    else:
      default_controllers = ["arm_left_controller", "gripper_left_controller", "arm_right_controller", "gripper_right_controller"]
  else: # TIAGo
    default_controllers = ["arm_controller", "gripper_controller"]

  rospy.wait_for_service("/controller_manager/list_controllers")
  rospy.wait_for_service("/controller_manager/switch_controller")

  manager_list = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
  manager_switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

  controllers = manager_list()

  start_controllers = default_controllers
  stop_controllers = [c.name for c in controllers.controller if c.name.startswith("spnav_controller") and c.state == "running"]

  rospy.loginfo("Switching controllers...")

  response = manager_switch(start_controllers=start_controllers, stop_controllers=stop_controllers, strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
