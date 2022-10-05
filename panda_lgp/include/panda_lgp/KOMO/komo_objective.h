#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
using namespace autodiff;

namespace KOMO_k2
{

typedef struct 
{
    uint num_phase_variables;
    uint num_phases;
    uint num_iterations;
}ObjectiveData;

double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data);    
real Objective(const ArrayXreal& x, const real& num_phase_variables);

} // namespace