#include <panda_lgp/KOMO/komo_objective.h>



namespace KOMO_k2
{


double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    ObjectiveData *d = reinterpret_cast<ObjectiveData*>(data);
    d->num_iterations++;   
    
    Eigen::VectorXd x_t = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    Eigen::VectorXd x_t_minus1(x.size());
    Eigen::VectorXd x_t_minus2(x.size());
    x_t_minus1 << x_t.head(d->num_phase_variables), x_t.head(x.size()-d->num_phase_variables);
    x_t_minus2 << x_t_minus1.head(d->num_phase_variables), x_t_minus1.head(x.size()-d->num_phase_variables);
    Eigen::VectorXd k_order = x_t + x_t_minus2 - 2*x_t_minus1;
    double cost = k_order.transpose()*k_order;

    if (!grad.empty())
    {
        std::fill(grad.begin(), grad.end(), 0);
        FillJacobianBlock(x, grad);
    }

    return cost;
}

void FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac)
{
    int _num_variables = 7;
    int _num_timesteps = 3;

    // row by row
    for (uint row=0; row<_num_variables; ++row)
    {
        jac[row] = 4*x[row] - 6*x[row+_num_variables] + 2*x[row+2*_num_variables];
        jac[row+_num_variables] =12*x[row+_num_variables] - 6*x[row] - 8*x[row+2*_num_variables] + 2*x[row+3*_num_variables];
        if (_num_timesteps == 2)
        {

        }
        else if (_num_timesteps == 3)
        {
            
        }
        else if (_num_timesteps > 4)
        {
            
        }
        
        for (uint t=2; t<_num_timesteps; ++t)
        {
            // jac[row+t*_num_variables] = 2*x[row-2*_num_variables] - 8*x[row-_num_variables] + 12*x[row] - 8*x[row+_num_variables] + 2*x[row+2*_num_variables];
        }
    }


    // for (int idx=0; idx<_num_variables; idx++)
    // {
    //     jac[idx]                   = 4*x[idx] - 6*x[idx+_num_variables] + 2*x[idx+2*_num_variables];
    //     jac[idx+_num_variables]    = 12*x[idx+_num_variables] - 6*x[idx] - 8*x[idx + 2*_num_variables] + 2*x[idx + 3*_num_variables];
    //     jac[idx+2*_num_variables]  = 2*x[idx]                     - 8*x[idx + _num_variables]    + 12*x[idx+ 2*_num_variables]  - 8*x[idx+3*_num_variables]  + 2*x[idx+4*_num_variables];
    //     jac[idx+3*_num_variables]  = 2*x[idx+ _num_variables]     - 8*x[idx + 2*_num_variables]  + 12*x[idx+ 3*_num_variables]  - 8*x[idx+4*_num_variables]  + 2*x[idx+5*_num_variables];
    //     jac[idx+4*_num_variables]  = 2*x[idx+ 2*_num_variables]   - 8*x[idx + 3*_num_variables]  + 12*x[idx+ 4*_num_variables]  - 8*x[idx+5*_num_variables]  + 2*x[idx+6*_num_variables];
    //     jac[idx+5*_num_variables]  = 2*x[idx+ 3*_num_variables]   - 8*x[idx + 4*_num_variables]  + 12*x[idx+ 5*_num_variables]  - 8*x[idx+6*_num_variables]  + 2*x[idx+7*_num_variables];
    //     jac[idx+6*_num_variables]  = 2*x[idx+ 4*_num_variables]   - 8*x[idx + 5*_num_variables]  + 12*x[idx+ 6*_num_variables]  - 8*x[idx+7*_num_variables]  + 2*x[idx+8*_num_variables];
    //     jac[idx+7*_num_variables]  = 2*x[idx+ 5*_num_variables]   - 8*x[idx + 6*_num_variables]  + 12*x[idx+ 7*_num_variables]  - 8*x[idx+8*_num_variables]  + 2*x[idx+9*_num_variables];
    //     jac[idx+8*_num_variables]  = 2*x[idx+ 6*_num_variables]   - 8*x[idx + 7*_num_variables]  + 12*x[idx+ 8*_num_variables]  - 8*x[idx+9*_num_variables]  + 2*x[idx+10*_num_variables];
    //     jac[idx+9*_num_variables]  = 2*x[idx+ 7*_num_variables]   - 8*x[idx + 8*_num_variables]  + 12*x[idx+ 9*_num_variables]  - 8*x[idx+10*_num_variables] + 2*x[idx+11*_num_variables];
    //     jac[idx+10*_num_variables] = 2*x[idx+ 8*_num_variables]  - 8*x[idx + 9*_num_variables]  + 12*x[idx+ 10*_num_variables] - 8*x[idx+11*_num_variables] + 2*x[idx+12*_num_variables];
    //     jac[idx+11*_num_variables] = 2*x[idx+ 9*_num_variables]  - 8*x[idx + 10*_num_variables] + 12*x[idx+ 11*_num_variables] - 8*x[idx+12*_num_variables] + 2*x[idx+13*_num_variables];
    //     jac[idx+12*_num_variables] = 2*x[idx+ 10*_num_variables] - 8*x[idx + 11*_num_variables] + 12*x[idx+ 12*_num_variables] - 8*x[idx+13*_num_variables] + 2*x[idx+14*_num_variables];
    //     jac[idx+13*_num_variables] = 2*x[idx+ 11*_num_variables] - 8*x[idx + 12*_num_variables] + 12*x[idx+ 13*_num_variables] - 8*x[idx+14*_num_variables] + 2*x[idx+15*_num_variables];
    //     jac[idx+14*_num_variables] = 2*x[idx+ 12*_num_variables] - 8*x[idx + 13*_num_variables] + 12*x[idx+ 14*_num_variables] - 8*x[idx+15*_num_variables] + 2*x[idx+16*_num_variables];
    //     jac[idx+15*_num_variables] = 2*x[idx+ 13*_num_variables] - 8*x[idx + 14*_num_variables] + 12*x[idx+ 15*_num_variables] - 8*x[idx+16*_num_variables] + 2*x[idx+17*_num_variables];
    //     jac[idx+16*_num_variables] = 2*x[idx+ 14*_num_variables] - 8*x[idx + 15*_num_variables] + 12*x[idx+ 16*_num_variables] - 8*x[idx+17*_num_variables] + 2*x[idx+18*_num_variables];
    //     jac[idx+17*_num_variables] = 2*x[idx+ 15*_num_variables] - 8*x[idx + 16*_num_variables] + 12*x[idx+ 17*_num_variables] - 8*x[idx+18*_num_variables] + 2*x[idx+19*_num_variables];
    //     jac[idx+18*_num_variables] = 2*x[idx+ 16*_num_variables] - 8*x[idx + 17*_num_variables] + 10*x[idx+ 18*_num_variables] - 4*x[idx+19*_num_variables];
    //     jac[idx+19*_num_variables] = 2*x[idx+ 17*_num_variables] - 4*x[idx + 18*_num_variables] + 2*x[idx+  19*_num_variables];
    // }
}


// double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data)
// {
//     ObjectiveData *d = reinterpret_cast<ObjectiveData*>(data);
//     d->num_iterations++;   

//     ArrayXreal x_diff = Eigen::Map<const Eigen::ArrayXd>(x.data(), x.size());

//     real num_phase_variables(d->num_phase_variables);
//     real F;
//     Eigen::VectorXd g = gradient(Objective, wrt(x_diff), at(x_diff, num_phase_variables), F); // evaluate the output vector F and the gradient F/dx

//     if (!grad.empty())
//     {
//         std::fill(grad.begin(), grad.end(), 0);
//         for (uint i=0; i<grad.size(); ++i)
//         {
//             grad[i] = g(i);
//         }
//     }

//     return F.val();
// }


real Objective(const ArrayXreal& x, const real& num_phase_variables)
{
    ArrayXreal x_t_minus1(x.size());
    x_t_minus1 << x.head(num_phase_variables.val()), x.head(x.size()-num_phase_variables.val());

    ArrayXreal x_t_minus2(x.size());
    x_t_minus2 << x_t_minus1.head(num_phase_variables.val()), x_t_minus1.head(x.size()-num_phase_variables.val());

    auto k_order = x + x_t_minus2 - 2*x_t_minus1;
    return (k_order*k_order).sum();
}


}

