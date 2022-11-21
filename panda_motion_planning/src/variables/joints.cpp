#include <panda_motion_planning/variables/joints.h>

JointsVariable::JointsVariable(const int num_variables, const int num_time_slices)
    : _q(num_variables, num_time_slices)
     ,_num_vector_variables(num_variables*num_time_slices)
     ,_num_variables(num_variables)
     ,_num_time_slices(num_time_slices)
     ,_lower_bounds(num_variables*num_time_slices)
     ,_upper_bounds(num_variables*num_time_slices)
{
    this->SetJointLimits();
}

JointsVariable::~JointsVariable()
{
}


std::vector<double> JointsVariable::GetLowerBounds()
{
    return _lower_bounds;
}

std::vector<double> JointsVariable::GetUpperBounds()
{
    return _upper_bounds;
}

void JointsVariable::SetJointLimits()
{
    for (size_t idx=0; idx<_num_time_slices; idx++)
    {
        // Set lower bounds for x
        _lower_bounds[idx*7+0] = -2.8973;
        _lower_bounds[idx*7+1] = -1.7628;
        _lower_bounds[idx*7+2] = -2.8973;
        _lower_bounds[idx*7+3] = -3.0718;
        _lower_bounds[idx*7+4] = -2.8973;
        _lower_bounds[idx*7+5] = -0.0175;
        _lower_bounds[idx*7+6] = -2.8973;
        // Set upper bounds for x
        _upper_bounds[idx*7+0] = 2.8973;
        _upper_bounds[idx*7+1] = 1.7628;
        _upper_bounds[idx*7+2] = 2.8973;
        _upper_bounds[idx*7+3] = -0.0698;
        _upper_bounds[idx*7+4] = 2.8973;
        _upper_bounds[idx*7+5] = 3.7525;
        _upper_bounds[idx*7+6] = 2.8973;
    }
}


void JointsVariable::SetBoundaryConstraints(const std::vector<double> start_state, const std::vector<double> goal_state)
{
    for (size_t idx=0; idx<_num_variables; idx++)
    {
        // Start state
        _lower_bounds[idx] = start_state[idx];
        _upper_bounds[idx] = start_state[idx];
        // Goal state
        _lower_bounds[(_num_time_slices-1)*_num_variables + idx] = goal_state[idx];
        _upper_bounds[(_num_time_slices-1)*_num_variables + idx] = goal_state[idx];
    }
}



std::vector<double> JointsVariable::InitialGuess(const std::vector<double> start_state, const std::vector<double> goal_state)
{
    std::vector<double> init(_num_variables*_num_time_slices);
    int idx = 0;


  // Initial guess
    for (int i=0; i<_num_variables; i++)
    {
        //  auto temp_init = linspace(q_start[i], q_goal[i], num_time_slices);
        auto temp_init = linspace(start_state[i], goal_state[i], _num_time_slices);
        _q.row(i) = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(temp_init.data(), temp_init.size());
        _q.col(_num_time_slices-1) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(goal_state.data(), goal_state.size());
    }


    for (int i=0; i<_q.cols(); i++)
    {
        for (int j=0; j<_q.rows(); j++)
        {
            init[idx] = _q(j, i);
            idx++;
        }
    }
    std::cout << "init = \n" << _q << std::endl;




    // for (size_t col=0; col<_num_time_slices; col++)
    // {
    //     for (size_t row=0; row<_num_variables; row++)
    //     {
    //         init[idx] = start_state[row];
    //         idx++;
    //     }
    // }

    // idx = _num_variables* (_num_time_slices-1);
    // for (int i=0; i<_num_variables; i++)
    // {
    //     init[idx] = goal_state[i];
    //     idx++;
    // }
    return init;
}

int JointsVariable::GetNumVariables()
{
    return _num_vector_variables;
}
