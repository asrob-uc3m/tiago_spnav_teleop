#ifndef __TIAGO_SPNAV_TELEOP_HPP__
#define __TIAGO_SPNAV_TELEOP_HPP__

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace tiago_controllers
{

class SpnavController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  hardware_interface::JointHandle joint_;
  static constexpr double gain_ = 1.25;
  static constexpr double setpoint_ = 3.00;
};

} //namespace

#endif // __TIAGO_SPNAV_TELEOP_HPP__
