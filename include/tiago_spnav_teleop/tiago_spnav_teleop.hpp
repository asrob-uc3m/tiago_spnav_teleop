#ifndef __TIAGO_SPNAV_TELEOP_HPP__
#define __TIAGO_SPNAV_TELEOP_HPP__

#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

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
  std::vector<hardware_interface::JointHandle> joints;
  KDL::Chain chain;
  KDL::ChainIkSolverVel_pinv ikSolverVel;
};

} //namespace

#endif // __TIAGO_SPNAV_TELEOP_HPP__
