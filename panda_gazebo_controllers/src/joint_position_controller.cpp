// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_gazebo_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_gazebo_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  _joint_position_goal_sub = node_handle.subscribe("/panda/joint_position_goal", 1, &JointPositionController::JointPositionGoalCallback, this);

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    _joint_position_initial[i] = position_joint_handles_[i].getPosition();
  }

  elapsed_time_ = ros::Duration(0.0);

  _joint_position_cmd = _joint_position_initial;
  _joint_position_goal = _joint_position_initial;
  _joint_position_prev = _joint_position_initial;
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  double distance_to_goal_point = 0;

  for (int i=0; i<_NUM_JOINTS; i++)
  {
    distance_to_goal_point = _joint_position_goal[i] - position_joint_handles_[i].getPosition();

    int direction = std::signbit(distance_to_goal_point)==1 ? -1 : 1;

    if (std::fabs(distance_to_goal_point)>_POSITION_ACCURACY)
    {
      _joint_position_cmd[i] += (direction * _JOINT_VELOCITY * period.toSec());
      position_joint_handles_[i].setCommand(_joint_position_cmd[i]);
    }
  }
}

void JointPositionController::JointPositionGoalCallback(const panda_gazebo_controllers::JointPosition::ConstPtr& msg)
{
  if (msg->joint_position.size() != _NUM_JOINTS)
  {
    ROS_ERROR_STREAM("PositionJointPositionController: Published Commands are not of size 7");
    _joint_position_goal = _joint_position_prev;
    _joint_position_cmd = _joint_position_prev;
  }
  else
  {
    std::copy(std::begin(msg->joint_position), std::end(msg->joint_position), std::begin(_joint_position_goal));
    // _joint_position_goal[0] = msg->joint_position[0];
    // _joint_position_goal[1] = msg->joint_position[1];
    // _joint_position_goal[2] = msg->joint_position[2];
    // _joint_position_goal[3] = msg->joint_position[3];
    // _joint_position_goal[4] = msg->joint_position[4];
    // _joint_position_goal[5] = msg->joint_position[5];
    // _joint_position_goal[6] = msg->joint_position[6];
  }
}

}  // namespace panda_gazebo_controllers

PLUGINLIB_EXPORT_CLASS(panda_gazebo_controllers::JointPositionController,
                       controller_interface::ControllerBase)