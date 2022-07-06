#pragma once

#include <ifopt/variable_set.h>
#include <Eigen/Dense>
#include <ifopt_test/tools_function.h>

namespace motion_planning {

using Vector2d = Eigen::Vector2d;


class DecisionVariables : public ifopt::VariableSet
{
public:
    // Every variable set has a name, here "var_set1". this allows the constraints
    // and costs to define values and Jacobians specifically w.r.t this variable set.
    // DecisionVariables();
    DecisionVariables(const std::string& name, const int num_variables, const int num_time_slices,
                      const std::vector<double> q_start, const std::vector<double> q_goal);

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..
    void SetVariables(const VectorXd& x) override;

    // Here is the reverse transformation from the internal representation to
    // to the Eigen::Vector
    VectorXd GetValues() const override;

    // Each variable has an upper and lower bound set here
    VecBound GetBounds() const override;

    Eigen::MatrixXd GetX0();

    // TODO:
    void InitialGuess();
    void AddStartGoalBound();

    int _num_variables;
    int _num_time_slices;

private:
    Eigen::MatrixXd _q;
    std::vector<double> _q_start = std::vector<double>(7);
    std::vector<double> _q_goal = std::vector<double>(7);

    Eigen::MatrixXd _upper_bounds_mat;
    Eigen::MatrixXd _lower_bounds_mat;

    
};

} // namespace ifopt