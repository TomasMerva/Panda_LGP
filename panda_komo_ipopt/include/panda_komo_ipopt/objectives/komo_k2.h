#pragma once

#include <ifopt/cost_term.h>


using Vector2d = Eigen::Vector2d;

class KOMO_k2_Objective : public ifopt::CostTerm
{
    public:
        // Objective();
        KOMO_k2_Objective(const std::string& name, const int num_variables, const int num_time_slices);

        double GetCost() const override;
        void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;
        void GetStateNodes(VectorXd &sym_set, Eigen::MatrixXd  &internal_set) const;

    private:
        int _num_variables;
        int _num_time_slices;
};

