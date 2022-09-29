#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>

namespace KOMO_k2
{

typedef struct 
{
    uint num_phase_variables;
    uint num_phases;
    uint num_iterations;
}ObjectiveData;

double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data);    
void FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac, ObjectiveData *data);
void GetStateNodes(const std::vector<double> &vector_set, Eigen::MatrixXd  &internal_rep_set);

} // namespace