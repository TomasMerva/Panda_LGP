#include <panda_inverse_kinematics/variables/joints.h>

JointVariables::JointVariables(const std::string& name, const int num_variables, const std::vector<double> q_start)
    : ifopt::VariableSet(num_variables, name)
{
    _q = Eigen::Map<const Eigen::VectorXd>(&q_start[0], q_start.size());
}


void JointVariables::SetVariables(const VectorXd& x)
{
    _q = x;
}

JointVariables::VectorXd JointVariables::GetValues() const
{
    return _q;
}

JointVariables::VecBound JointVariables::GetBounds() const
{
    VecBound bounds(GetRows());
    bounds.at(0) = ifopt::Bounds(-2.8973, 2.8973);  // joint_1
    bounds.at(1) = ifopt::Bounds(-1.7628, 1.7628);  // joint_2
    bounds.at(2) = ifopt::Bounds(-2.8973, 2.8973);  // joint_3
    bounds.at(3) = ifopt::Bounds(-3.0718, -0.0698); // joint_4
    bounds.at(4) = ifopt::Bounds(-2.8973, 2.8973);  // joint_5
    bounds.at(5) = ifopt::Bounds(-0.0175, 3.7525);  // joint_6
    bounds.at(6) = ifopt::Bounds(-2.8973, 2.8973);  // joint_7
    return bounds;
}
