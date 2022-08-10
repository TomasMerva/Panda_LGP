#pragma once

#include <ifopt/cost_term.h>


using Vector2d = Eigen::Vector2d;

class QuadraticErrorCost : public ifopt::CostTerm
{
    public:
        // Objective();
        QuadraticErrorCost(const std::string& name, const int num_variables, const std::vector<double> q_des);

        double GetCost() const override;
        void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;

    private:
        int _num_variables;
        Eigen::VectorXd _q_desired;
};

