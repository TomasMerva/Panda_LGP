#include <panda_inverse_kinematics/constraints/position_constraint.h>

std::vector<double> PositionConstraint::_p_AQ_lower;
std::vector<double> PositionConstraint::_p_AQ_upper;

PositionConstraint::PositionConstraint(const std::string& name, std::vector<double> p_AQ_lower, std::vector<double> p_AQ_upper)
    : ifopt::ConstraintSet(3, name)
{
    _p_AQ_lower = p_AQ_lower;
    _p_AQ_upper = p_AQ_upper;
}

Eigen::VectorXd PositionConstraint::GetValues() const
{
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();
    auto FK_q = Kinematics::ForwardKinematics(x, true);
    Eigen::VectorXd p_AQ(3);
    p_AQ << FK_q(0,3), FK_q(1,3), FK_q(2,3);
    for (size_t idx=0; idx<3; idx++)
    {
        g(idx) = p_AQ(idx);
    }
    return g;
}

PositionConstraint::VecBound PositionConstraint::GetBounds() const
{
    VecBound bounds(GetRows());
    for (size_t idx=0; idx<3; idx++)
    {
        bounds[idx] = ifopt::Bounds(_p_AQ_lower[idx], _p_AQ_upper[idx]);
    }
    return bounds;
}

void PositionConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac_block) const
{
    if (var_set == "x") 
    {
        VectorXd x = GetVariables()->GetComponent("x")->GetValues();

        double delta_q = 0.001;
        auto FK_q = Kinematics::ForwardKinematics(x, true);
        Eigen::VectorXd p_AQ(3);
        p_AQ << FK_q(0,3), FK_q(1,3), FK_q(2,3);

        Eigen::MatrixXd J(3, 7);
        for (int i=0; i<7; i++)
        {
            auto temp = x;
            temp(i) += delta_q; // add delta_q to the q(i)
            auto FK_with_deltaq = Kinematics::ForwardKinematics(temp, true); //compute FK with q(i)+delta_q
            jac_block.coeffRef(0, i) = (FK_with_deltaq(0,3) - FK_q(0,3)) / delta_q;  // dx / dq(i)
            jac_block.coeffRef(1, i) = (FK_with_deltaq(1,3) - FK_q(1,3)) / delta_q;  // dy / dq(i)
            jac_block.coeffRef(2, i) = (FK_with_deltaq(2,3) - FK_q(2,3)) / delta_q;  // dz / dq(i)
        }
    }
}