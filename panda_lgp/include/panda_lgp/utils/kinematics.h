#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>

// Parameters are set for Franka Emika Panda

namespace kinematics
{

struct JointLimits
{
    // <low limit, upper limit>
    std::pair<double, double> q1_limit = std::make_pair(-2.8973, 2.8973);
    std::pair<double, double> q2_limit = std::make_pair(-1.7628, 1.7628);
    std::pair<double, double> q3_limit = std::make_pair(-2.8973, 2.8973);
    std::pair<double, double> q4_limit = std::make_pair(-3.0718, -0.0698);
    std::pair<double, double> q5_limit = std::make_pair(-2.8973, 2.8973);
    std::pair<double, double> q6_limit = std::make_pair(-0.0175, 3.7525);
    std::pair<double, double> q7_limit = std::make_pair(-2.8973, 2.8973);
};

// TODO: I would like to add here maybe frames or something. Have to decide what is useful
struct Configuration
{
    double q_dim = 7;
    double frame_dim = 3; // TODO: not the best way of doing it
    std::vector<double> q_act = std::vector<double>(q_dim);
    JointLimits joint_limits;
};

Eigen::Matrix4d DH_matrix(const double a, const double d, 
                          const double alpha, const double theta);

Eigen::Matrix4d ForwardKinematics(Eigen::VectorXd q, bool gripper_enable);

Eigen::MatrixXd GeometricJacobian(Eigen::VectorXd q, bool gripper_enable);                     



} // namespace

