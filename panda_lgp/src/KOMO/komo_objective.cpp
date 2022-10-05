#include <panda_lgp/KOMO/komo_objective.h>



namespace KOMO_k2
{


double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    ObjectiveData *d = reinterpret_cast<ObjectiveData*>(data);
    d->num_iterations++;   

    ArrayXreal x_diff = Eigen::Map<const Eigen::ArrayXd>(x.data(), x.size());
    std::cout << x_diff.transpose()  << "\n" << std::endl;

    real num_phase_variables(d->num_phase_variables);
    real F;
    Eigen::VectorXd g = gradient(Objective, wrt(x_diff), at(x_diff, num_phase_variables), F); // evaluate the output vector F and the gradient F/dx

    if (!grad.empty())
    {
        std::fill(grad.begin(), grad.end(), 0);
        for (uint i=0; i<grad.size(); ++i)
        {
            grad[i] = g(i);
        }
    }

    return F.val();
}


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

