#ifndef __TIAGO_SPNAV_TELEOP_HPP__
#define __TIAGO_SPNAV_TELEOP_HPP__

#include <array>
#include <mutex>
#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/Joy.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>

namespace tiago_controllers
{

class SpnavController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  SpnavController();
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  void spnavCallback(const sensor_msgs::Joy::ConstPtr& msg);

  std::vector<hardware_interface::JointHandle> armJoints;
  std::vector<hardware_interface::JointHandle> gripperJoints;
  std::vector<double> armJointUpperLimits;
  std::vector<double> armJointLowerLimits;
  std::array<float, 6> joyAxes;
  std::array<int, 2> joyButtons;
  KDL::Chain chain;
  KDL::ChainIkSolverVel_pinv ikSolverVel;
  KDL::JntArray q;
  ros::Subscriber spnav;
  std::mutex mtx;
};

} //namespace

#endif // __TIAGO_SPNAV_TELEOP_HPP__
