#include <panda_komo_ipopt/objectives/komo_k2.h>


KOMO_k2_Objective::KOMO_k2_Objective(const std::string& name, const int num_variables, const int num_time_slices)
    : ifopt::CostTerm(name)
    , _num_variables(num_variables)
    , _num_time_slices(num_time_slices)
{
}


void KOMO_k2_Objective::GetStateNodes(VectorXd &sym_set, Eigen::MatrixXd &internal_set) const
{
    size_t idx = 0;
    for (size_t j=0; j<internal_set.cols(); j++)
    {
        for (size_t i=0; i<internal_set.rows(); i++)
        {
            internal_set(i, j) = sym_set(idx);
            idx++;
        }
    }
}

double KOMO_k2_Objective::GetCost() const
{
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();

    // Eigen::MatrixXd x_t(_num_variables, _num_time_slices);
    // GetStateNodes(x, x_t);
    
    // double cost = 0.0;
    // // First col is zero
    // // -----
    // // Second column
    // size_t idx = 1;
    // auto temp = x_t.col(idx) - x_t.col(idx-1);
    // cost += temp.transpose()*temp;

    // // Rest of columns
    // idx++;
    // for (idx; idx<_num_time_slices; idx++)
    // {
    //     auto temp = x_t.col(idx) - 2*x_t.col(idx-1) + x_t.col(idx-2);
    //     cost += temp.transpose()*temp;
    //     // std::cout << "cost: " << cost << " ";

    // }
    // return cost;    

    ArrayXreal x_diff = Eigen::Map<Eigen::ArrayXd>(x.data(), x.size());

    real cost;
    auto g = gradient(Objective, wrt(x_diff), at(x_diff), cost); // evaluate the output vector F and the gradient F/dx
    return cost.val();
}

real KOMO_k2_Objective::Objective(const ArrayXreal& x)
{
    uint num_variables = 7;

    ArrayXreal x_t_minus1(x.size());
    x_t_minus1 << x.head(num_variables), x.head(x.size()-num_variables);

    ArrayXreal x_t_minus2(x.size());
    x_t_minus2 << x_t_minus1.head(num_variables), x_t_minus1.head(x.size()-num_variables);

    auto k_order = x + x_t_minus2 - 2*x_t_minus1;
    return (k_order*k_order).sum();
}

void KOMO_k2_Objective::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
    // if (var_set == "x") {
    //     VectorXd x = GetVariables()->GetComponent("x")->GetValues();

    //     for (int idx=0; idx<_num_variables; idx++)
    //     {
    //         jac.coeffRef(0, idx)                  = 4*x(idx) - 6*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
    //         jac.coeffRef(0, idx+_num_variables)   = 12*x(idx+_num_variables) - 6*x(idx) - 8*x(idx + 2*_num_variables) + 2*x(idx + 3*_num_variables);
    //         jac.coeffRef(0, idx+2*_num_variables) = 2*x(idx)                     - 8*x(idx + _num_variables)    + 12*x(idx+ 2*_num_variables)  - 8*x(idx+3*_num_variables)  + 2*x(idx+4*_num_variables);
    //         jac.coeffRef(0, idx+3*_num_variables) = 2*x(idx+ _num_variables)     - 8*x(idx + 2*_num_variables)  + 12*x(idx+ 3*_num_variables)  - 8*x(idx+4*_num_variables)  + 2*x(idx+5*_num_variables);
    //         jac.coeffRef(0, idx+4*_num_variables) = 2*x(idx+ 2*_num_variables)   - 8*x(idx + 3*_num_variables)  + 12*x(idx+ 4*_num_variables)  - 8*x(idx+5*_num_variables)  + 2*x(idx+6*_num_variables);
    //         jac.coeffRef(0, idx+5*_num_variables) = 2*x(idx+ 3*_num_variables)   - 8*x(idx + 4*_num_variables)  + 12*x(idx+ 5*_num_variables)  - 8*x(idx+6*_num_variables)  + 2*x(idx+7*_num_variables);
    //         jac.coeffRef(0, idx+6*_num_variables) = 2*x(idx+ 4*_num_variables)   - 8*x(idx + 5*_num_variables)  + 12*x(idx+ 6*_num_variables)  - 8*x(idx+7*_num_variables)  + 2*x(idx+8*_num_variables);
    //         jac.coeffRef(0, idx+7*_num_variables) = 2*x(idx+ 5*_num_variables)   - 8*x(idx + 6*_num_variables)  + 12*x(idx+ 7*_num_variables)  - 8*x(idx+8*_num_variables)  + 2*x(idx+9*_num_variables);
    //         jac.coeffRef(0, idx+8*_num_variables) = 2*x(idx+ 6*_num_variables)   - 8*x(idx + 7*_num_variables)  + 12*x(idx+ 8*_num_variables)  - 8*x(idx+9*_num_variables)  + 2*x(idx+10*_num_variables);
    //         jac.coeffRef(0, idx+9*_num_variables) = 2*x(idx+ 7*_num_variables)   - 8*x(idx + 8*_num_variables)  + 12*x(idx+ 9*_num_variables)  - 8*x(idx+10*_num_variables) + 2*x(idx+11*_num_variables);
    //         jac.coeffRef(0, idx+10*_num_variables) = 2*x(idx+ 8*_num_variables)  - 8*x(idx + 9*_num_variables)  + 12*x(idx+ 10*_num_variables) - 8*x(idx+11*_num_variables) + 2*x(idx+12*_num_variables);
    //         jac.coeffRef(0, idx+11*_num_variables) = 2*x(idx+ 9*_num_variables)  - 8*x(idx + 10*_num_variables) + 12*x(idx+ 11*_num_variables) - 8*x(idx+12*_num_variables) + 2*x(idx+13*_num_variables);
    //         jac.coeffRef(0, idx+12*_num_variables) = 2*x(idx+ 10*_num_variables) - 8*x(idx + 11*_num_variables) + 12*x(idx+ 12*_num_variables) - 8*x(idx+13*_num_variables) + 2*x(idx+14*_num_variables);
    //         jac.coeffRef(0, idx+13*_num_variables) = 2*x(idx+ 11*_num_variables) - 8*x(idx + 12*_num_variables) + 12*x(idx+ 13*_num_variables) - 8*x(idx+14*_num_variables) + 2*x(idx+15*_num_variables);
    //         jac.coeffRef(0, idx+14*_num_variables) = 2*x(idx+ 12*_num_variables) - 8*x(idx + 13*_num_variables) + 12*x(idx+ 14*_num_variables) - 8*x(idx+15*_num_variables) + 2*x(idx+16*_num_variables);
    //         jac.coeffRef(0, idx+15*_num_variables) = 2*x(idx+ 13*_num_variables) - 8*x(idx + 14*_num_variables) + 12*x(idx+ 15*_num_variables) - 8*x(idx+16*_num_variables) + 2*x(idx+17*_num_variables);
    //         jac.coeffRef(0, idx+16*_num_variables) = 2*x(idx+ 14*_num_variables) - 8*x(idx + 15*_num_variables) + 12*x(idx+ 16*_num_variables) - 8*x(idx+17*_num_variables) + 2*x(idx+18*_num_variables);
    //         jac.coeffRef(0, idx+17*_num_variables) = 2*x(idx+ 15*_num_variables) - 8*x(idx + 16*_num_variables) + 12*x(idx+ 17*_num_variables) - 8*x(idx+18*_num_variables) + 2*x(idx+19*_num_variables);
    //         jac.coeffRef(0, idx+18*_num_variables) = 2*x(idx+ 16*_num_variables) - 8*x(idx + 17*_num_variables) + 10*x(idx+ 18*_num_variables) - 4*x(idx+19*_num_variables);
    //         jac.coeffRef(0, idx+19*_num_variables) = 2*x(idx+ 17*_num_variables) - 4*x(idx + 18*_num_variables) + 2*x(idx+  19*_num_variables);
    //     }
    // }
        if (var_set == "x") {

        VectorXd x = GetVariables()->GetComponent("x")->GetValues();

        ArrayXreal x_diff = Eigen::Map<Eigen::ArrayXd>(x.data(), x.size());
        real cost;
        
        auto g = gradient(Objective, wrt(x_diff), at(x_diff), cost); // evaluate the output vector F and the gradient F/dx
        for (uint idx=0; idx<x.size(); ++idx)
        {
            jac.coeffRef(0, idx) = g(idx);
        }
        }
}

