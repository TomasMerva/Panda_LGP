#include <panda_lgp/KOMO/komo_objective.h>



namespace KOMO_k2
{


double 
GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data)
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
        FillJacobianBlock(x, grad, d->num_phase_variables, d->num_phases);
    }

    return cost;
}

void 
FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac, 
                        const uint num_variables, const uint num_timestesp)
{
    // row by row
    for (uint row=0; row<num_variables; ++row)
    {
        if (num_timestesp == 2)
        {
            jac[row] = 2*x[row] - 2*x[row+num_variables];
            jac[row + num_variables] = 2*x[row+num_variables] - 2*x[row];
        }
        else if (num_timestesp == 3)
        {
            jac[row] = 4*x[row] - 6*x[row + num_variables] + 2*x[row + 2*num_variables];
            jac[row + num_variables] = 10*x[row + num_variables] - 6*x[row] - 4*x[row + 2*num_variables];
            jac[row + 2*num_variables] = 2*x[row] - 4*x[row + num_variables] + 2*x[row + 2*num_variables];
        }
        else if (num_timestesp == 4)
        {
            jac[row] = 4*x[row] - 6*x[row + num_variables] + 2*x[row + 2*num_variables];
            jac[row + num_variables] = 12*x[row + num_variables] - 6*x[row] - 8*x[row + 2*num_variables] + 2*x[row + 3*num_variables];
            jac[row + 2*num_variables] = 2*x[row] - 8*x[row + num_variables] + 10*x[row + 2*num_variables] - 4*x[row + 3*num_variables];
            jac[row + 3*num_variables] = 2*x[row + num_variables] - 4*x[row + 2*num_variables] + 2*x[row + 3*num_variables];
        }
        else if (num_variables > 4)
        {
            jac[row] = 4*x[row] - 6*x[row + num_variables] + 2*x[row + 2*num_variables];
            jac[row + num_variables] = 12*x[row + num_variables] - 6*x[row] - 8*x[row + 2*num_variables] + 2*x[row + 3*num_variables];
            uint t=0;
            for (t; t<(num_timestesp - 4); ++t)   // this row starts when num_variables > 4
            {
                jac[row + (t+2)*num_variables] = 2*x[row + t*num_variables] - 8*x[row + (t+1)*num_variables] + 12*x[row + (t+2)*num_variables] - 8*x[row + (t+3)*num_variables] + 2*x[row + + (t+4)*num_variables];
            }
            jac[row + (num_timestesp-2)*num_variables] = 2*x[row + (t)*num_variables] - 8*x[row + (t+1)*num_variables] + 10*x[row + (t+2)*num_variables] - 4*x[row + (t+3)*num_variables];
            jac[row + (num_timestesp-1)*num_variables] = 2*x[row + (t+1)*num_variables] - 4*x[row + (t+2)*num_variables] + 2*x[row + (t+3)*num_variables];
        }
    }
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


// real Objective(const ArrayXreal& x, const real& num_phase_variables)
// {
//     ArrayXreal x_t_minus1(x.size());
//     x_t_minus1 << x.head(num_phase_variables.val()), x.head(x.size()-num_phase_variables.val());

//     ArrayXreal x_t_minus2(x.size());
//     x_t_minus2 << x_t_minus1.head(num_phase_variables.val()), x_t_minus1.head(x.size()-num_phase_variables.val());

//     auto k_order = x + x_t_minus2 - 2*x_t_minus1;
//     return (k_order*k_order).sum();
// }


}

