#include "tiago_spnav_teleop/tiago_spnav_teleop.hpp"

#include <algorithm> // std::copy
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>

using namespace tiago_controllers;

constexpr auto JOY_GRIPPER_INCREMENT = 0.001;
constexpr auto UPDATE_LOG_THROTTLE = 1.0; // [s]

SpnavController::SpnavController()
    : ikSolverVel(nullptr)
{ }

SpnavController::~SpnavController()
{
    if (ikSolverVel)
    {
        delete ikSolverVel;
        ikSolverVel = nullptr;
    }
}

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

    q.resize(chain.getNrOfJoints());

    double eps;
    int maxIter;

    if (!n.getParam("ik_solver_vel_eps", eps))
    {
        ROS_ERROR("Could not find ik_solver_vel_eps parameter");
        return false;
    }

    if (!n.getParam("ik_solver_vel_max_iter", maxIter))
    {
        ROS_ERROR("Could not find ik_solver_vel_max_iter parameter");
        return false;
    }

    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, eps, maxIter);

    urdf::Model model;

    if (!model.initString(robot_desc_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    std::vector<std::string> arm_joint_names;

    if (!n.getParam("arm_joint_names", arm_joint_names))
    {
        ROS_ERROR("Could not retrieve arm joint names");
        return false;
    }

    for (const auto & joint_name : arm_joint_names)
    {
        armJoints.push_back(hw->getHandle(joint_name));
        armJointLowerLimits.push_back(model.getJoint(joint_name)->limits->lower);
        armJointUpperLimits.push_back(model.getJoint(joint_name)->limits->upper);
    }

    std::vector<std::string> gripper_joint_names;

    if (!n.getParam("gripper_joint_names", gripper_joint_names))
    {
        ROS_ERROR("Could not retrieve gripper joint names");
        return false;
    }

    for (const auto & joint_name : gripper_joint_names)
    {
        gripperJoints.push_back(hw->getHandle(joint_name));
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
    KDL::JntArray qdot(chain.getNrOfJoints());
    KDL::Twist tw;

    {
        std::lock_guard<std::mutex> lock(mtx);
        tw.vel = {joyAxes[0], joyAxes[1], joyAxes[2]};
        tw.rot = {joyAxes[3], joyAxes[4], joyAxes[5]};
    }

    if (!checkReturnCode(ikSolverVel->CartToJnt(q, tw, qdot)))
    {
        return;
    }

    KDL::JntArray q_temp(q); // look ahead in case we may have ended up in a singular point at `q`

    for (int i = 0; i < armJoints.size(); i++)
    {
        q_temp(i) += qdot(i) * period.toSec();
    }

    KDL::JntArray qdot_temp(chain.getNrOfJoints());

    if (!checkReturnCode(ikSolverVel->CartToJnt(q_temp, tw, qdot_temp)))
    {
        return;
    }

    q = q_temp; // no singular point, so update the calculated pose

    for (int i = 0; i < armJoints.size(); i++)
    {
        armJoints[i].setCommand(q(i));
    }

    if (joyButtons[0])
    {
        for (int i = 0; i < gripperJoints.size(); i++)
        {
            gripperJoints[i].setCommand(gripperJoints[i].getPosition() + JOY_GRIPPER_INCREMENT);
        }
    }
    else if (joyButtons[1])
    {
        for (int i = 0; i < gripperJoints.size(); i++)
        {
            gripperJoints[i].setCommand(gripperJoints[i].getPosition() - JOY_GRIPPER_INCREMENT);
        }
    }
}

bool SpnavController::checkReturnCode(int ret)
{
    switch (ret)
    {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Convergence issue: pseudo-inverse is singular");
        return false;
    case KDL::SolverI::E_SVD_FAILED:
        ROS_ERROR_THROTTLE(UPDATE_LOG_THROTTLE, "Convergence issue: SVD failed");
        return false;
    case KDL::SolverI::E_NOERROR:
        return true;
    default:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Convergence issue: unknown error");
        return false;
    }
}

void SpnavController::starting(const ros::Time& time)
{
    for (int i = 0; i < armJoints.size(); i++)
    {
        q(i) = armJoints[i].getPosition();
    }

    ROS_INFO("Initial arm pose: %f %f %f %f %f %f %f", q(0), q(1), q(2), q(3), q(4), q(5), q(6));
}

void SpnavController::stopping(const ros::Time& time)
{ }

void SpnavController::spnavCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    std::copy(msg->axes.cbegin(), msg->axes.cend(), joyAxes.begin());
    std::copy(msg->buttons.cbegin(), msg->buttons.cend(), joyButtons.begin());
}

PLUGINLIB_EXPORT_CLASS(tiago_controllers::SpnavController, controller_interface::ControllerBase);
