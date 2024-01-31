#include "tiago_spnav_teleop/tiago_spnav_teleop.hpp"

#include <algorithm> // std::copy
#include <string>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>

using namespace tiago_controllers;

constexpr auto UPDATE_LOG_THROTTLE = 1.0; // [s]

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

    std::string start_link, end_link;

    if (!n.getParam("start_link", start_link))
    {
        ROS_ERROR("Could not find start_link parameter");
        return false;
    }

    if (!n.getParam("end_link", end_link))
    {
        ROS_ERROR("Could not find end_link parameter");
        return false;
    }

    if (!tree.getChain(start_link, end_link, chain))
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

    if (!n.getParam("joy_arm_scale", joyArmScale))
    {
        ROS_ERROR("Could not retrieve joy arm scale");
        return false;
    }

    if (!n.getParam("joy_gripper_increment", joyGripperIncrement))
    {
        ROS_ERROR("Could not retrieve joy gripper increment");
        return false;
    }

    for (const auto & joint_name : arm_joint_names)
    {
        armJoints.push_back(hw->getHandle(joint_name));

        armJointLimits.emplace_back(model.getJoint(joint_name)->limits->lower,
                                    model.getJoint(joint_name)->limits->upper);
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
        q_temp(i) += qdot(i) * period.toSec() * joyArmScale;

        if (q_temp(i) < armJointLimits[i].first || q_temp(i) > armJointLimits[i].second)
        {
            ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Joint %d out of limits: %f not in [%f, %f]",
                                                   i, q_temp(i), armJointLimits[i].first, armJointLimits[i].second);
            return;
        }
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
            gripperJoints[i].setCommand(gripperJoints[i].getPosition() + joyGripperIncrement);
        }
    }
    else if (joyButtons[1])
    {
        for (int i = 0; i < gripperJoints.size(); i++)
        {
            gripperJoints[i].setCommand(gripperJoints[i].getPosition() - joyGripperIncrement);
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
    std::string out = "Initial arm pose:";

    for (int i = 0; i < armJoints.size(); i++)
    {
        q(i) = armJoints[i].getPosition();
        out += " " + std::to_string(q(i));
    }

    ROS_INFO("Initial arm pose: %s", out.c_str());
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
