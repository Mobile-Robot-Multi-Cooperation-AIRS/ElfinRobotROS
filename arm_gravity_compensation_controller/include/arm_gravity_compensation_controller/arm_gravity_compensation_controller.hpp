/*
 * @Author: yaojianming youjianmin123@qq.com
 * @LastEditors: yaojianming
 * @LastEditTime: 2024-04-30 10:34:55
 * @Description: 
 */
#pragma once
#include <forward_command_controller/forward_joint_group_command_controller.h>
namespace arm_gravity_compensation_controller
{
  /***
   * @brief torque compensation for gravity.
   * @param mass
   * @b command null 
  */
// typedef forward_command_controller::ForwardJointGroupCommandController<hardware_interface::EffortJointInterface>
//   ArmGravityCompensationController;
class ArmGravityCompensationController
  : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ArmGravityCompensationController() {}
  ~ArmGravityCompensationController() {sub_command_.shutdown();}

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

  //From ForwardJointGroupCommandController Member
  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
  unsigned int n_joints_;

private:
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  //From ForwardJointGroupCommandController Member
  ros::Subscriber sub_command_;
};

}


