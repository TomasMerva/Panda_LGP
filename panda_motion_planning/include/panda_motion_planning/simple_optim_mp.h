#pragma once

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>


namespace ifopt {
using Eigen::Vector2d;
using Eigen::VectorXd;

class ExVariables : public VariableSet {
  public:
    // Every variable set has a name, here "var_set1". this allows the constraints
    // and costs to define values and Jacobians specifically w.r.t this variable set.
    ExVariables() : ExVariables("decision_variable") {};
    ExVariables(const std::string& name) : VariableSet(2, name)
    {
      // the initial values where the NLP starts iterating from
      // x0_ = 3.5;
      // x1_ = -3.5;
    }

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..
    void SetVariables(const VectorXd& x) override
    {
      // x0_ = x(0);
      // x1_ = x(1);
    };

    // // Here is the reverse transformation from the internal representation to
    // // to the Eigen::Vector
    VectorXd GetValues() const override
    {
      // return Vector3d(x0_, x1_);
    };

    // Each variable has an upper and lower bound set here
    VecBound GetBounds() const override
    {
      VecBound bounds(GetRows());
      std::cout << bounds.at(0) << std::endl;
      // bounds.at(0) = Bounds(-5.0, 5.0);
      // bounds.at(1) = Bounds(-5.0, 5.0);
      return bounds;
    }

  private:
    int num_collocation_points = 10;
    int num_states = 3;
    // position, velocity and force
    VectorXd x_ = VectorXd(num_collocation_points*num_states);
};


// class ExCost: public CostTerm {
//   public:
//     ExCost() : ExCost("cost_term1") {}
//     ExCost(const std::string& name) : CostTerm(name) {}

//     double GetCost() const override
//     {
//       Vector2d x = GetVariables()->GetComponent("joints")->GetValues();
//       // return -std::pow(x(1)-2, 2);
//       return std::pow(x(0)+1, 2) + std::pow(x(1), 2);
//     };

//     void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
//     {
//       if (var_set == "joints") {
//         Vector2d x = GetVariables()->GetComponent("joints")->GetValues();
//         // jac.coeffRef(0, 0) = 0.0;             // derivative of cost w.r.t x0
//         // jac.coeffRef(0, 1) = -2.0*(x(1)-2.0); // derivative of cost w.r.t x1
//         jac.coeffRef(0, 0) = 2*(x(0)+1);             // derivative of cost w.r.t x0
//         jac.coeffRef(0, 1) = 2*x(1);                // derivative of cost w.r.t x1
//       }
//     }
// };

} // namespace ifopt