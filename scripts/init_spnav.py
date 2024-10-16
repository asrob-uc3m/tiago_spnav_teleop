#!/usr/bin/env python

import argparse
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState, Joy
from controller_manager_msgs.srv import SwitchController

if __name__ == "__main__":
  rospy.init_node("init_spnav")

  parser = argparse.ArgumentParser()
  parser.add_argument("--arm", type=str, default=None)
  args, unknown = parser.parse_known_args()

  if args.arm is not None: # TIAGo++
    if args.arm not in ["left", "right", "both"]:
      rospy.logfatal("Invalid argument for --arm: %s" % args.arm)
      exit(1)

    if args.arm == "left":
      limb = "left arm"
      motion_name = "extend_left"
      default_controllers = ["arm_left_controller", "gripper_left_controller"]
      spnav_controllers = ["spnav_controller_left"]
    elif args.arm == "right":
      limb = "right arm"
      motion_name = "extend_right"
      default_controllers = ["arm_right_controller", "gripper_right_controller"]
      spnav_controllers = ["spnav_controller_right"]
    elif args.arm == "both":
      limb = "both arms"
      motion_name = "extend"
      default_controllers = ["arm_left_controller", "gripper_left_controller", "arm_right_controller", "gripper_right_controller"]
      spnav_controllers = ["spnav_controller_left", "spnav_controller_right"]
  else: # TIAGo
    limb = "arm"
    motion_name = "extend"
    default_controllers = ["arm_controller", "gripper_controller"]
    spnav_controllers = ["spnav_controller"]

  rospy.loginfo("Waiting for play_motion...")
  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("/joint_states", JointState)
  rospy.sleep(3.0)

  rospy.loginfo("Extend %s..." % limb)
  goal = PlayMotionGoal()
  goal.motion_name = motion_name
  goal.skip_planning = False

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("Arm extended.")

  for controller in spnav_controllers:
    rospy.wait_for_message("%s/spacenav/joy" % controller, Joy)

  rospy.wait_for_service("/controller_manager/switch_controller")
  manager = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
  rospy.loginfo("Switching controllers...")
  response = manager(start_controllers=spnav_controllers, stop_controllers=default_controllers, strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
