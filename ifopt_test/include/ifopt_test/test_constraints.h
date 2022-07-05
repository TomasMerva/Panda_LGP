#pragma once

#include <ifopt/constraint_set.h>


namespace ifopt {
using Vector2d = Eigen::Vector2d;


class Constraints : public ConstraintSet
{
public:
    Constraints();
    Constraints(const std::string& name, const int num_variables);

    VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;


};

} //namespace