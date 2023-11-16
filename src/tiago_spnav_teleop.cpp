#include "tiago_spnav_teleop/tiago_spnav_teleop.hpp"

#include <algorithm> // std::copy
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

    std::vector<std::string> joint_names;

    if (!n.getParam("joint_names", joint_names))
    {
        ROS_ERROR("Could not retrieve joint names");
        return false;
    }

    for (const auto & joint_name : joint_names)
    {
        joints.push_back(hw->getHandle(joint_name));
    }

    ros::topic::waitForMessage<sensor_msgs::Joy>("/spacenav/joy", n);
    spnav = n.subscribe("/spacenav/joy", 1, &SpnavController::spnavCallback, this);

    if (!spnav)
    {
        ROS_ERROR("Could not subscribe to /spacenav/joy");
        return false;
    }

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

void SpnavController::spnavCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    std::copy(msg->axes.cbegin(), msg->axes.cend(), joyAxes.begin());
    std::copy(msg->buttons.cbegin(), msg->buttons.cend(), joyButtons.begin());
}

PLUGINLIB_EXPORT_CLASS(tiago_controllers::SpnavController, controller_interface::ControllerBase);
