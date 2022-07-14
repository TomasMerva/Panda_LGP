#pragma once
#include <vector>
#include <math.h>
#include <Eigen/Dense>

class KOMO_k2
{
    public:
        KOMO_k2(const int num_variables, const int num_time_slices);
        ~KOMO_k2();

        static double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);
        static void FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac);
        static void GetStateNodes(const std::vector<double> &vector_set, Eigen::MatrixXd  &internal_rep_set);

        static int num_iterations;

    private:
        static const int _num_variables;
        static const int _num_time_slices;

}; 

