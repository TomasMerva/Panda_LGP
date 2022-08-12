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
    Eigen::MatrixXd R_WA = FK_q.block<3,3>(0,0);    // Rotation matrix of the current q configuration
    auto R_WB = Kinematics::RotationMatrixFromRPY(_rpy[0], _rpy[1], _rpy[2]); // The desired rotation matrix
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
    if (var_set == "x") 
    {
        VectorXd x = GetVariables()->GetComponent("x")->GetValues();
        auto FK_q = Kinematics::ForwardKinematics(x, true);
        Eigen::MatrixXd R_WA = FK_q.block<3,3>(0,0);
        auto R_WB = Kinematics::RotationMatrixFromRPY(_rpy[0], _rpy[1], _rpy[2]);
        auto R_AB = R_WA.transpose() * R_WB;
        
        const Eigen::Vector3d r_AB{R_AB(1, 2) - R_AB(2, 1), R_AB(2, 0) - R_AB(0, 2),
                                   R_AB(0, 1) - R_AB(1, 0)};
        Eigen::MatrixXd Jq_w_AB(3, 7);


        double delta_q = 0.001;
        for (int i=0; i<7; i++)
        {
            auto temp = x;
            temp(i) += delta_q; // ad delta_q to the q(i)
            auto FK_with_deltaq = Kinematics::ForwardKinematics(temp, true); //compute FK with q(i)+delta_q
            Eigen::MatrixXd R_WA_with_deltaq = FK_with_deltaq.block<3,3>(0,0);
            auto R_AB = R_WA.transpose() * R_WB;

            Jq_w_AB(0, i) = (FK_with_deltaq(0,3) - FK_q(0,3)) / delta_q;  // dx / dq(i)
            Jq_w_AB(1, i) = (FK_with_deltaq(1,3) - FK_q(1,3)) / delta_q;  // dy / dq(i)
            Jq_w_AB(2, i) = (FK_with_deltaq(2,3) - FK_q(2,3)) / delta_q;  // dz / dq(i)
        }

    }
}