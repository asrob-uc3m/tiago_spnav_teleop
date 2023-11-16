#include "tiago_spnav_teleop/tiago_spnav_teleop.hpp"

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>

using namespace tiago_controllers;

SpnavController::SpnavController()
    : ikSolverVel(chain)
{ }

bool SpnavController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    KDL::Tree tree;
    std::string robot_desc_string;

    if (!n.getParam("/robot_description", robot_desc_string))
    {
        ROS_ERROR("Could not find robot_description");
        return false;
    }

    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    if (!tree.getChain("torso_lift_link", "arm_7_link", chain))
    {
        ROS_ERROR("Failed to get chain from kdl tree");
        return false;
    }

    ROS_INFO("Got chain with %d joints and %d segments", chain.getNrOfJoints(), chain.getNrOfSegments());

    ikSolverVel.updateInternalDataStructures();

    // get joint name from the parameter server
    std::string my_joint;

    if (!n.getParam("joint", my_joint))
    {
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
