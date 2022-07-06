#include <ifopt_test/test_constraints.h>

namespace ifopt {

Constraints::Constraints()
    : Constraints("constraint1", 1)
{
}

Constraints::Constraints(const std::string& name, const int num_variables)
    : ConstraintSet(num_variables, name)
{  
}

Constraints::VectorXd Constraints::GetValues() const
{
    VectorXd g(GetRows());
    Vector2d x = GetVariables()->GetComponent("x")->GetValues();
    g(0) = std::pow(x(0),2) + x(1);
    return g;
}

Constraints::VecBound Constraints::GetBounds() const 
{
    Constraints::VecBound b(GetRows());
    b.at(0) = Bounds(1.0, 1.0);
    return b;
}


void Constraints::FillJacobianBlock (std::string var_set, Jacobian& jac_block) const
{
    if (var_set == "x") {
      Vector2d x = GetVariables()->GetComponent("x")->GetValues();

      jac_block.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
      jac_block.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
    }
}

} // ifopt namespace
