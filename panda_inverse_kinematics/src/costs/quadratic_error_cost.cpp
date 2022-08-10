#include <panda_inverse_kinematics/costs/quadratic_error_cost.h>
#include <iostream>
QuadraticErrorCost::QuadraticErrorCost(const std::string& name, const int num_variables, const std::vector<double> q_des)
    : ifopt::CostTerm(name)
    , _num_variables(num_variables)
{
    _q_desired = Eigen::Map<const Eigen::VectorXd>(&q_des[0], q_des.size());
}

double QuadraticErrorCost::GetCost() const
{
    Eigen::VectorXd x = GetVariables()->GetComponent("x")->GetValues();
    auto Q = Eigen::MatrixXd::Identity(_num_variables, _num_variables);
    double cost = (x-_q_desired).transpose() * Q * (x-_q_desired);
    return cost;
}

void QuadraticErrorCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
    if (var_set == "x") 
    {
        Eigen::VectorXd x = GetVariables()->GetComponent("x")->GetValues();        
        for (int idx=0; idx<_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*(x[idx] - _q_desired[idx]);
        }
    }
}