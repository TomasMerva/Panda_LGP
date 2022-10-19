#pragma once
#include <vector>
#include <Eigen/Dense>
#include <panda_lgp/utils/kinematics.h>
#include <cmath>
#include <iostream>

namespace Constraint
{

enum ConstraintSymbol{
    CS_AxisInRegion,
    FS_AxisInRegion,
    FS_ObjectInRegion,
    FS_PickFixedFrame,
    FS_Grasp,
    FS_none,
    FS_PointToPointDistance,
    FS_FixedOrientationAxis
};

typedef struct
{
    uint num_phase_variables;
    uint idx;
    Eigen::Matrix4d frame_A;
    Eigen::Matrix4d frame_B;
    std::vector<double> region = std::vector<double>(2);
    double tolerance;
}ConstraintData;


// Constraints
// double AxisInRegion(const std::vector<double> &x, std::vector<double> &grad, void *data);
// double ManipulationFrame(const std::vector<double> &x, std::vector<double> &grad, void *data);
void ManipulationFrame(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);
void AxisInRegion(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);


void Zaxis(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);


// Add Constraint Data for skeleton
// ConstraintData AddAxisInRegionData(uint idx, std::string region);


} // namespace

