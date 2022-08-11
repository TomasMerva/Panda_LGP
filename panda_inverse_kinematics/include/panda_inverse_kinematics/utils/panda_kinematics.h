#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Kinematics{


// Forward Kinematics
Eigen::Matrix4d DH_matrix(const double a, const double d, 
                        const double alpha, const double theta);
Eigen::Matrix4d ForwardKinematics(Eigen::VectorXd q, bool gripper_enable);

Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd q, const Eigen::Matrix4d FK_q);

// Inverse Kinematics
std::array<double, 7> InverseKinematics(std::array<double, 16> O_T_EE_array,
                                    double q7, std::array<double, 7> q_actual_array);


// Rz*Ry*Rx 
Eigen::Matrix3d RotationMatrixFromRPY(double roll, double pitch, double yaw);

} //namespace panda_kinematics

