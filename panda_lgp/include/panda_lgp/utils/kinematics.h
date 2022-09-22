#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>


namespace kinematics
{


Eigen::Matrix4d DH_matrix(const double a, const double d, 
                          const double alpha, const double theta);

Eigen::Matrix4d ForwardKinematics(Eigen::VectorXd q, bool gripper_enable);

Eigen::MatrixXd GeometricJacobian(Eigen::VectorXd q, bool gripper_enable);                     



} // namespace