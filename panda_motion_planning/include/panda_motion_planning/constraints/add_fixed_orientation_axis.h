#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <panda_motion_planning/utils/panda_kinematics.h>


typedef struct{
    size_t idx;
    size_t axis_idx;
    double axis_vector_1;
    double axis_vector_2;
    double axis_vector_3;
}AddFixedOrientationAxisData;

namespace Constraint
{
    double AddFixedOrientationAxis(const std::vector<double> &x, std::vector<double> &grad, void *data);
}