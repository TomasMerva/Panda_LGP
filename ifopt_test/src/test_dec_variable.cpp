#include <ifopt_test/test_dec_variable.h>

namespace motion_planning {


// DecisionVariables::DecisionVariables()
//     : DecisionVariables("x", 7, 20, std::vector<float>{7,0.0}, std::vector<float>{7, 0.0})
// {
// }

DecisionVariables::DecisionVariables(const std::string& name, const int num_variables, const int num_time_slices,
                                    const std::vector<float> q_start, const std::vector<float> q_goal)
    : ifopt::VariableSet(num_variables*num_time_slices, name)
      ,_q(num_variables, num_time_slices)
    //   ,_upper_bounds_mat(num_variables, num_time_slices)
    //   ,_lower_bounds_mat(num_variables, num_time_slices)

{
    _q_start = q_start;
    _q_goal = q_goal;
    _num_variables = num_variables;
    _num_time_slices = num_time_slices;

    // Initial guess
    for (int i=0; i<num_variables; i++)
    {
        // auto temp_init = linspace(q_start[i], q_goal[i], num_time_slices);
        std::vector<double> temp_init(num_time_slices, _q_start[i]);
        _q.row(i) = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(temp_init.data(), temp_init.size());
    }
}


void DecisionVariables::SetVariables(const VectorXd& x)
{
    int idx = 0;
    for (int j=0; j<_q.cols(); j++)
    {
        for (int i=0; i<_q.rows(); i++)
        {
            _q(i,j) = x(idx);
            idx++;
        }
    }
}


DecisionVariables::VectorXd DecisionVariables::GetValues() const
{
    VectorXd x(GetRows());
    int idx = 0;
    for (int j=0; j<_q.cols(); j++)
    {
        for (int i=0; i<_q.rows(); i++)
        {
            x(idx) = _q(i,j);
            idx++;
        }
    }
    return x;
} 


DecisionVariables::VecBound DecisionVariables::GetBounds() const
{
    VecBound bounds(GetRows());
    for (int i=0; i<_num_time_slices; i++)
    {
        bounds.at(i*7+0) = ifopt::Bounds(-2.8973, 2.8973);  // joint_1
        bounds.at(i*7+1) = ifopt::Bounds(-1.7628, 1.7628);  // joint_2
        bounds.at(i*7+2) = ifopt::Bounds(-2.8973, 2.8973);  // joint_3
        bounds.at(i*7+3) = ifopt::Bounds(-3.0718, -0.0698); // joint_4
        bounds.at(i*7+4) = ifopt::Bounds(-2.8973, 2.8973);  // joint_5
        bounds.at(i*7+5) = ifopt::Bounds(-0.0175, 3.7525);  // joint_6
        bounds.at(i*7+6) = ifopt::Bounds(-2.8973, 2.8973);  // joint_7
    }

    // start
    int idx = 0;
    for (int i=0; i<_num_variables; i++)
    {
        bounds.at(i) = ifopt::Bounds(_q_start[i], _q_start[i]);
        bounds.at( (_num_time_slices-1)*_num_variables  + idx) =  ifopt::Bounds(_q_goal[i], _q_goal[i]);
        idx++;
    }
    return bounds;
}

Eigen::MatrixXd DecisionVariables::GetX0()
{
    return _q;
}

} //ifopt namespace