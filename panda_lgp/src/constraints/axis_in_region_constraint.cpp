#include <panda_lgp/constraints/constraints.h>


// Euclidean distance between [y] and middle of the region
double Constraint::AxisInRegion(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(data);

    Eigen::Map<const Eigen::VectorXd> q(x.data()+g_data->idx*g_data->num_phase_variables, g_data->num_phase_variables);
    // Return Jacobian and FK
    auto kin_conf = kinematics::GeometricJacobian(q, true);
    auto J = kin_conf[0];
    auto FK = kin_conf[1];
    double middle_region = g_data->region[0] - (g_data->region[0] - g_data->region[1])/2.0;

    double diff = FK(1,3) - middle_region;
    double l2_norm = std::sqrt(std::pow(diff, 2))-middle_region;

    if (!grad.empty())
    {
        std::fill(grad.begin(), grad.end(), 0);
        Eigen::VectorXd dg = Eigen::VectorXd::Zero(g_data->num_phase_variables);
        dg.head(7) = (diff * J.row(1)) / l2_norm;
        for (int i=0; i<g_data->num_phase_variables; ++i)
        {
            grad[i + g_data->idx * g_data->num_phase_variables] = dg(i);
        }
    }
    return l2_norm;
}

