// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <panda_gazebo_controllers/JointPosition.h>

namespace panda_gazebo_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                        hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void JointPositionGoalCallback(const panda_gazebo_controllers::JointPosition::ConstPtr& msg);

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};

  ros::Subscriber _joint_position_goal_sub;
  std::array<double, 7> _joint_position_initial{};
  std::array<double, 7> _joint_position_goal{};
  std::array<double, 7> _joint_position_cmd{};
  std::array<double, 7> _joint_position_prev{};

  const int _NUM_JOINTS = 7;
  const double _POSITION_ACCURACY = 0.001;
  const double _JOINT_VELOCITY = 0.1;
};

}  // namespace panda_gazebo_controllers