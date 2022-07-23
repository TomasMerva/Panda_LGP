#include <panda_motion_planning/objective/komo_k2.h>

int KOMO_k2::num_iterations = 0;
const int KOMO_k2::_num_variables = 7;
const int KOMO_k2::_num_time_slices = 20;

KOMO_k2::KOMO_k2(const int num_variables, const int num_time_slices)
{
   
}

KOMO_k2::~KOMO_k2()
{
}


void KOMO_k2::GetStateNodes(const std::vector<double> &vector_set, Eigen::MatrixXd  &internal_rep_set)
{
    size_t idx = 0;
    for (size_t j=0; j<internal_rep_set.cols(); j++)
    {
        for (size_t i=0; i<internal_rep_set.rows(); i++)
        {
            internal_rep_set(i, j) = vector_set[idx];
            idx++;
        }
    }
}


double KOMO_k2::GetCost(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    num_iterations++;

    if (!grad.empty()) {
        std::fill(grad.begin(), grad.end(), 0.0);
        FillJacobianBlock(x, grad);
    }

    Eigen::MatrixXd x_t(_num_variables, _num_time_slices);
    GetStateNodes(x, x_t);

    double cost = 0.0;
    // First col is zero
    // -----

    // Second column
    size_t idx = 1;
    auto temp = x_t.col(idx) - x_t.col(idx-1);
    cost += temp.transpose()*temp;

    // Rest of columns
    idx++;
    for (idx; idx<_num_time_slices; idx++)
    {
        auto temp = x_t.col(idx) - 2*x_t.col(idx-1) + x_t.col(idx-2);
        cost += temp.transpose()*temp;
    }
    return cost;    
}

void KOMO_k2::FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac)
{
    for (int idx=0; idx<_num_variables; idx++)
    {
        jac[idx]                   = 4*x[idx] - 6*x[idx+_num_variables] + 2*x[idx+2*_num_variables];
        jac[idx+_num_variables]    = 12*x[idx+_num_variables] - 6*x[idx] - 8*x[idx + 2*_num_variables] + 2*x[idx + 3*_num_variables];
        jac[idx+2*_num_variables]  = 2*x[idx]                     - 8*x[idx + _num_variables]    + 12*x[idx+ 2*_num_variables]  - 8*x[idx+3*_num_variables]  + 2*x[idx+4*_num_variables];
        jac[idx+3*_num_variables]  = 2*x[idx+ _num_variables]     - 8*x[idx + 2*_num_variables]  + 12*x[idx+ 3*_num_variables]  - 8*x[idx+4*_num_variables]  + 2*x[idx+5*_num_variables];
        jac[idx+4*_num_variables]  = 2*x[idx+ 2*_num_variables]   - 8*x[idx + 3*_num_variables]  + 12*x[idx+ 4*_num_variables]  - 8*x[idx+5*_num_variables]  + 2*x[idx+6*_num_variables];
        jac[idx+5*_num_variables]  = 2*x[idx+ 3*_num_variables]   - 8*x[idx + 4*_num_variables]  + 12*x[idx+ 5*_num_variables]  - 8*x[idx+6*_num_variables]  + 2*x[idx+7*_num_variables];
        jac[idx+6*_num_variables]  = 2*x[idx+ 4*_num_variables]   - 8*x[idx + 5*_num_variables]  + 12*x[idx+ 6*_num_variables]  - 8*x[idx+7*_num_variables]  + 2*x[idx+8*_num_variables];
        jac[idx+7*_num_variables]  = 2*x[idx+ 5*_num_variables]   - 8*x[idx + 6*_num_variables]  + 12*x[idx+ 7*_num_variables]  - 8*x[idx+8*_num_variables]  + 2*x[idx+9*_num_variables];
        jac[idx+8*_num_variables]  = 2*x[idx+ 6*_num_variables]   - 8*x[idx + 7*_num_variables]  + 12*x[idx+ 8*_num_variables]  - 8*x[idx+9*_num_variables]  + 2*x[idx+10*_num_variables];
        jac[idx+9*_num_variables]  = 2*x[idx+ 7*_num_variables]   - 8*x[idx + 8*_num_variables]  + 12*x[idx+ 9*_num_variables]  - 8*x[idx+10*_num_variables] + 2*x[idx+11*_num_variables];
        jac[idx+10*_num_variables] = 2*x[idx+ 8*_num_variables]  - 8*x[idx + 9*_num_variables]  + 12*x[idx+ 10*_num_variables] - 8*x[idx+11*_num_variables] + 2*x[idx+12*_num_variables];
        jac[idx+11*_num_variables] = 2*x[idx+ 9*_num_variables]  - 8*x[idx + 10*_num_variables] + 12*x[idx+ 11*_num_variables] - 8*x[idx+12*_num_variables] + 2*x[idx+13*_num_variables];
        jac[idx+12*_num_variables] = 2*x[idx+ 10*_num_variables] - 8*x[idx + 11*_num_variables] + 12*x[idx+ 12*_num_variables] - 8*x[idx+13*_num_variables] + 2*x[idx+14*_num_variables];
        jac[idx+13*_num_variables] = 2*x[idx+ 11*_num_variables] - 8*x[idx + 12*_num_variables] + 12*x[idx+ 13*_num_variables] - 8*x[idx+14*_num_variables] + 2*x[idx+15*_num_variables];
        jac[idx+14*_num_variables] = 2*x[idx+ 12*_num_variables] - 8*x[idx + 13*_num_variables] + 12*x[idx+ 14*_num_variables] - 8*x[idx+15*_num_variables] + 2*x[idx+16*_num_variables];
        jac[idx+15*_num_variables] = 2*x[idx+ 13*_num_variables] - 8*x[idx + 14*_num_variables] + 12*x[idx+ 15*_num_variables] - 8*x[idx+16*_num_variables] + 2*x[idx+17*_num_variables];
        jac[idx+16*_num_variables] = 2*x[idx+ 14*_num_variables] - 8*x[idx + 15*_num_variables] + 12*x[idx+ 16*_num_variables] - 8*x[idx+17*_num_variables] + 2*x[idx+18*_num_variables];
        jac[idx+17*_num_variables] = 2*x[idx+ 15*_num_variables] - 8*x[idx + 16*_num_variables] + 12*x[idx+ 17*_num_variables] - 8*x[idx+18*_num_variables] + 2*x[idx+19*_num_variables];
        jac[idx+18*_num_variables] = 2*x[idx+ 16*_num_variables] - 8*x[idx + 17*_num_variables] + 10*x[idx+ 18*_num_variables] - 4*x[idx+19*_num_variables];
        jac[idx+19*_num_variables] = 2*x[idx+ 17*_num_variables] - 4*x[idx + 18*_num_variables] + 2*x[idx+  19*_num_variables];
    }
}
