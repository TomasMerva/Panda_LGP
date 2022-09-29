#include <panda_lgp/constraints/constraints.h>

namespace Constraint
{

// Euclidean distance between [y] and middle of the region
double AxisInRegion(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    ConstraintData *g_data = reinterpret_cast<ConstraintData*>(data);

    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+g_data->idx*g_data->x_dim, g_data->x_dim);
    // returns Jacobian and FK
    auto kin_conf = kinematics::GeometricJacobian(q, true);
    auto J = kin_conf[0];
    auto FK = kin_conf[1];

    double middle_region = g_data->region[0] - (g_data->region[0] - g_data->region[1])/2.0;
    std::cout << "middle_region: " << middle_region << std::endl;

    double diff = FK(1,3) - middle_region;
    double l2_norm = std::sqrt(std::pow(diff, 2));

    if (!grad.empty())
    {
        std::fill(grad.begin(), grad.end(), 0);
        auto dg = diff * J.row(1) / l2_norm;
        for (int i=0; i<g_data->x_dim; ++i)
        {
            grad[i + g_data->idx * g_data->x_dim] = dg(i);
        }
    }
    return l2_norm;
}

} // namespace