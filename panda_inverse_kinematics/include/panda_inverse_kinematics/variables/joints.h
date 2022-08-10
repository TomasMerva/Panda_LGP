#pragma once

#include <ifopt/variable_set.h>
#include <Eigen/Dense>
#include <iostream>

using Vector2d = Eigen::Vector2d;


class JointVariables : public ifopt::VariableSet
{
public:
    // Every variable set has a name, here "var_set1". this allows the constraints
    // and costs to define values and Jacobians specifically w.r.t this variable set.
    // DecisionVariables();
    JointVariables(const std::string& name, const int num_variables, const std::vector<double> q_start);

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..
    void SetVariables(const VectorXd& x) override;

    // Here is the reverse transformation from the internal representation to
    // to the Eigen::Vector
    VectorXd GetValues() const override;

    // Each variable has an upper and lower bound set here
    VecBound GetBounds() const override;


private:
    Eigen::VectorXd _q;
    int _num_variables;

};
