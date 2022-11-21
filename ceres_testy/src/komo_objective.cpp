#include "ceres_testy/komo_objective.h"

KOMO_FeatureCost::KOMO_FeatureCost()
{
    set_num_residuals(3);
    *mutable_parameter_block_sizes() = std::vector<int32_t>(3, 3); // always 3 PB with 3 variables [x,y,z]
}

bool 
KOMO_FeatureCost::Evaluate(double const* const* parameters,
                           double* residuals,
                           double** jacobians) const
{
    residuals[0] = parameters[0][0] - 2.0*parameters[1][0] + parameters[2][0];
    residuals[1] = parameters[0][1] - 2.0*parameters[1][1] + parameters[2][1];
    residuals[2] = parameters[0][2] - 2.0*parameters[1][2] + parameters[2][2];

    // Compute the Jacobian if asked for.
    if (jacobians != nullptr && jacobians[0] != nullptr && jacobians[1] != nullptr && jacobians[2] != nullptr) {
        // jacobians[id_parameterblocku][id_res*block_size + id_var]
        memset(jacobians[0], 0, 3*3*sizeof(jacobians[0]));
        memset(jacobians[1], 0, 3*3*sizeof(jacobians[1]));
        memset(jacobians[2], 0, 3*3*sizeof(jacobians[2]));
        // res[0]
        jacobians[0][0] = 1.0;
        jacobians[1][0] = -2.0;
        jacobians[2][0] = 1.0;
        // res[1]
        jacobians[0][4] = 1.0;
        jacobians[1][4] = -2.0;
        jacobians[2][4] = 1.0;
        // res[2]
        jacobians[0][8] = 1.0;
        jacobians[1][8] = -2.0;
        jacobians[2][8] = 1.0;
    }
    return true;
}