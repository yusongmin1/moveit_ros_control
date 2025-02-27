//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include "std_msgs/Float64MultiArray.h"
#include <atomic>
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>
#include <chrono>
#include <hardware_interface/joint_command_interface.h>

namespace UR5
{

  using namespace hardware_interface;
  class UR5CONTROLLER
      : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
  {
  public:
    UR5CONTROLLER() = default;
    ~UR5CONTROLLER() override;
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void starting(const ros::Time &time) override;
    std::vector<JointHandle> jointHandles_;

    int count=0;
  };

} // namespace legged