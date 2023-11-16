#include "tiago_spnav_teleop/tiago_spnav_teleop.hpp"

#include <pluginlib/class_list_macros.h>

using namespace tiago_controllers;

bool SpnavController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    // get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
    return true;
}

void SpnavController::update(const ros::Time& time, const ros::Duration& period)
{
    // double error = setpoint_ - joint_.getPosition();
    // joint_.setCommand(error*gain_);
}

void SpnavController::starting(const ros::Time& time)
{ }
void SpnavController::stopping(const ros::Time& time)
{ }

PLUGINLIB_EXPORT_CLASS(tiago_controllers::SpnavController, controller_interface::ControllerBase);
