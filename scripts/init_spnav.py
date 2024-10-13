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
  parser.add_argument("--dual", type=str, default=None)
  args, unknown = parser.parse_known_args()

  if args.dual is not None:
    if args.dual != "left" and args.dual != "right":
      rospy.logfatal("Invalid argument for --dual: %s" % args.dual)
      exit(1)

    limb = args.dual + " arm"
    motion_name = "extend_" + args.dual
    default_controllers = ["arm_%s_controller" % args.dual, "gripper_%s_controller" % args.dual]
  else:
    limb = "arm"
    motion_name = "extend"
    default_controllers = ["arm_controller", "gripper_controller"]

  rospy.loginfo("Waiting for play_motion...")
  client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("joint_states", JointState)
  rospy.sleep(3.0)

  rospy.loginfo("Extend %s..." % limb)
  goal = PlayMotionGoal()
  goal.motion_name = motion_name
  goal.skip_planning = False

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("Arm extended.")

  rospy.wait_for_message("/spacenav/joy", Joy)
  rospy.wait_for_service("controller_manager/switch_controller")
  manager = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
  rospy.loginfo("Switching controllers...")
  response = manager(start_controllers=["spnav_controller"], stop_controllers=default_controllers, strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
