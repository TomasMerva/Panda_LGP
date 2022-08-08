#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <panda_motion_planning/utils/panda_kinematics.h>


typedef struct {
    size_t idx;
    double obj_pos_x;
    double obj_pos_y;
    double obj_pos_z;
    double tolerance;
} AddPointToPointDistanceData;

namespace Constraint
{
    double AddPointToPointDistanceConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);
    
}

