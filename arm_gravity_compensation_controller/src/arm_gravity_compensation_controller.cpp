/*
 * @Author: yaojianming youjianmin123@qq.com
 * @LastEditors: yaojianming
 * @LastEditTime: 2024-04-30 11:05:59
 * @Description: 
 */
#include <arm_gravity_compensation_controller/arm_gravity_compensation_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace arm_gravity_compensation_controller
{
/**
 * @brief 控制器开始时设置
*/
void ArmGravityCompensationController::starting(const ros::Time& time)
{
  ROS_INFO_STREAM("Starting ArmGravityCompensationController");
  commands_buffer_.readFromRT()->assign(n_joints_,0.0);
}
/**
 * @brief 控制器初始化
 * 
 * @param hw 
 * @param n 
 * @return ArmGravityCompensationController::bool 
 */
bool ArmGravityCompensationController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{
  // List of controlled joints
  std::string param_name = "joints";
  if(!n.getParam(param_name, joint_names_))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  n_joints_ = joint_names_.size();

  if(n_joints_ == 0){
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }
  for(unsigned int i=0; i<n_joints_; i++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }
  commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
  sub_command_ = 
    n.subscribe<std_msgs::Float64MultiArray>("command", 1, 
      &ArmGravityCompensationController::commandCB, this);

  return true;
}
/**
 * @brief 控制器更新内容
 * 
 */

void ArmGravityCompensationController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  for(unsigned int i=0; i<n_joints_; i++)
  {  joints_[i].setCommand(commands[i]);  }
}

void ArmGravityCompensationController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  if(msg->data.size()!=n_joints_)
  {
    ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
    return;
  }
  commands_buffer_.writeFromNonRT(msg->data);
}

}


PLUGINLIB_EXPORT_CLASS(arm_gravity_compensation_controller::ArmGravityCompensationController,
  controller_interface::ControllerBase)