//
// Created by feidedao
//

#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/Float64MultiArray.h"
#include <ur5_ros_control/ur5_controllers.h>
namespace UR5
{
  bool UR5CONTROLLER::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
  {

    ros::NodeHandle nh;

    // Hardware interface
    auto *JointInterface = robot_hw->get<PositionJointInterface>();
    auto joint_names={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    for (const auto &jointName : joint_names)
    {
      jointHandles_.push_back(JointInterface->getHandle(jointName));
    }
    return true;
  }

  void UR5CONTROLLER::starting(const ros::Time &time)
  {

  }

  void UR5CONTROLLER::update(const ros::Time &time, const ros::Duration &period)
  {
    for(int i=0;i<6;i++)
    {
      if(i==0)
      jointHandles_[i].setCommand(count++);
      else jointHandles_[i].setCommand(0);
    }
  }



  UR5CONTROLLER::~UR5CONTROLLER()
  {

    std::cerr << "########################################################################";
    std::cerr << "\n### STOP UR5CONTROLLER";
    std::cerr << "########################################################################";
  }


 
} // namespace legged
PLUGINLIB_EXPORT_CLASS(UR5::UR5CONTROLLER, controller_interface::ControllerBase)
// PLUGINLIB_EXPORT_CLASS(legged::LeggedMpcController, controller_interface::ControllerBase)