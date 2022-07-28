#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <panda_motion_planning/utils/panda_kinematics.h>

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

typedef struct {
    int idx = 0;
    double obj_pos_x;
    double obj_pos_y;
    double obj_pos_z;
    double tolerance = 0.1;
} AddPointToPointDistanceData;

namespace Constraint
{
    double AddPointToPointDistanceConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);
    
}

namespace ConstraintTools
{
    autodiff::MatrixXreal DH_matrix(const autodiff::real a, const autodiff::real d, 
                                    const autodiff::real alpha, const autodiff::real theta);
    static autodiff::real ComputePointToPointDistanceConstraint (autodiff::VectorXreal q, autodiff::VectorXreal obj_position);     
}