#include <panda_inverse_kinematics/constraints/orientation_constraint.h>

double OrientationConstraint::_theta_bound;
std::vector<double> OrientationConstraint::_rpy;

OrientationConstraint::OrientationConstraint(const std::string& name, const std::vector<double> rpy, const double theta_bound)
    : ifopt::ConstraintSet(1, name)
{
   _theta_bound = theta_bound;
   _rpy = rpy;
}

Eigen::VectorXd OrientationConstraint::GetValues() const
{
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();
    auto FK_q = Kinematics::ForwardKinematics(x, true);
    Eigen::MatrixXd R_WA = FK_q.block<3,3>(0,0);
    auto R_WB = Kinematics::RotationMatrixFromRPY(_rpy[0], _rpy[1], _rpy[2]);
    auto R_AB = R_WA.transpose() * R_WB;
    g(0) = R_AB.trace();
    return g;
}

OrientationConstraint::VecBound OrientationConstraint::GetBounds() const
{
    VecBound bounds(GetRows());
    double g_low = 2*cos(_theta_bound)+1;
    bounds[0] = ifopt::Bounds(g_low, ifopt::inf);
    return bounds;
}

void OrientationConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac_block) const
{

}