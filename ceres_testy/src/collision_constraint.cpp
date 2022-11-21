#include "ceres_testy/collision_constraint.h"

Collision_FeatureCost::Collision_FeatureCost(std::vector<double> obstacle_position)
{
    set_num_residuals(1);
    *mutable_parameter_block_sizes() = std::vector<int32_t>(1, 3); // always 3 PB with 3 variables [x,y,z]
    _obstacle_pos = obstacle_position;
}

bool
Collision_FeatureCost::Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const
{   
    double g = -sqrt( (parameters[0][0] - _obstacle_pos[0])*(parameters[0][0] - _obstacle_pos[0])
                     +(parameters[0][1] - _obstacle_pos[1])*(parameters[0][1] - _obstacle_pos[1])
                     +(parameters[0][2] - _obstacle_pos[2])*(parameters[0][2] - _obstacle_pos[2]) ) + 0.707;
    if (g>0)
    {
        residuals[0] = g;
        // std::cout << "[" << parameters[0][0] << ", " << parameters[0][1] << ", " << parameters[0][2] << "] = " << g << "\n\n";
        if (jacobians != nullptr && jacobians[0] != nullptr) 
        {
            // jacobians[id_parameterblocku][id_res*block_size + id_var]
            jacobians[0][0] = -2*parameters[0][0];
            jacobians[0][1] = -2*parameters[0][1];
            jacobians[0][2] = -2*parameters[0][2];
        }
    }
    else
    {
        residuals[0] = 0.0;
        if (jacobians != nullptr && jacobians[0] != nullptr) 
        {
        memset(jacobians[0], 0, 3*sizeof(jacobians[0]));
        }
    }   
    return true;
}

std::vector<double> Collision_Langrange::lambda = std::vector<double>(20, 0);

//------------------
Collision_Langrange::Collision_Langrange(const int idx, std::vector<double> obstacle_position)
{
    set_num_residuals(1);
    *mutable_parameter_block_sizes() = std::vector<int32_t>(1, 3); // always 3 PB with 3 variables [x,y,z]
    _obstacle_pos = obstacle_position;
    _idx = idx;
}

bool
Collision_Langrange::Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const
{   
    double g = -sqrt( (parameters[0][0] - _obstacle_pos[0])*(parameters[0][0] - _obstacle_pos[0])
                     +(parameters[0][1] - _obstacle_pos[1])*(parameters[0][1] - _obstacle_pos[1])
                     +(parameters[0][2] - _obstacle_pos[2])*(parameters[0][2] - _obstacle_pos[2]) ) + 0.707;
    residuals[0] = lambda[_idx]*g;
    if (jacobians != nullptr && jacobians[0] != nullptr) 
    {
        jacobians[0][0] = lambda[_idx]*(-2*parameters[0][0]);
        jacobians[0][1] = lambda[_idx]*(-2*parameters[0][1]);
        jacobians[0][2] = lambda[_idx]*(-2*parameters[0][2]);
    }
    return true;
}