#include <panda_lgp/constraints/constraints.h>


// Euclidean distance between [y] and middle of the region
double Constraint::AxisInRegion(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(data);    
    double middle_region = g_data->region[0] - (g_data->region[0] - g_data->region[1])/2.0;
    std::cout << g_data->idx << std::endl;
    double diff = x[8 + g_data->idx*g_data->num_phase_variables] - middle_region;
    double g = std::sqrt( std::pow(diff,2) );

    if (!grad.empty())
    {
        std::fill(grad.begin(), grad.end(), 0);
        grad[8 + g_data->idx*g_data->num_phase_variables] = diff/g;
    }
    return g;
}

