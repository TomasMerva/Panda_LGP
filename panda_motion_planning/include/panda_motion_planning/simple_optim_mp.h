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
    ExVariables() : ExVariables("joints") {};
    ExVariables(const std::string& name) : VariableSet(num_collocation_points*num_states, name)
    {
      for (int i=0; i<x_.size(); ++i)
      {
        x_(i) = 0;
      }
    }

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..
    void SetVariables(const VectorXd& x) override
    {
      x_ = x;
    };

    // // Here is the reverse transformation from the internal representation to
    // // to the Eigen::Vector
    VectorXd GetValues() const override
    {
      return x_;
    };

    // Each variable has an upper and lower bound set here
    VecBound GetBounds() const override
    {
      VecBound bounds(num_collocation_points*num_states);
      //Joint_2 limit
      for (int i=0; i<num_collocation_points; i++)
      {
        bounds.at(i) = Bounds(-1.5707, 0.1745);
      }
      //Joint_3 limit
      for (int i=num_collocation_points; i<(num_collocation_points*2); i++)
      {
        bounds.at(i) = Bounds(0, 2.3561);
      }
      //Joint_5 limit
      for (int i= (2*num_collocation_points); i<(num_collocation_points*3); i++)
      {
        bounds.at(i) = Bounds(-1.5707, 1.9198);
      }

      // Init state
      bounds.at(0) = Bounds(-1.5707, -1.5707);
      bounds.at(10) = Bounds(1.5707, 1.57);
      bounds.at(20) = Bounds(1.5707, 1.57);
      // Final state
      bounds.at(0) = Bounds(-0.6204, -0.6204);
      bounds.at(10) = Bounds(1.7688, 1.7688);
      bounds.at(20) = Bounds(1.1719, 1.1719);
      return bounds;
    }

  private:
    int num_collocation_points = 10;
    int num_states = 3;

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