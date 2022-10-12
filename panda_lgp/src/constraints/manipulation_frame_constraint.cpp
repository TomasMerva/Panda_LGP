#include <panda_lgp/constraints/constraints.h>

// double 
// Constraint::ManipulationFrame(const std::vector<double> &x, std::vector<double> &grad, void *data)
// {
//     Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(data);


//     // Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
//     Eigen::Map<const Eigen::VectorXd> q(x.data()+g_data->idx*g_data->num_phase_variables, g_data->num_phase_variables);
//     auto kin_conf = kinematics::GeometricJacobian(q, true);
//     Eigen::MatrixXd J = kin_conf[0];
//     Eigen::MatrixXd FK = kin_conf[1];
//     // std::vector<double> RPY = kinematics::ConvertToRPY(FK);
    
//     double g = FK(0,3) - q(7);
//     if (!grad.empty())
//     {
//         std::fill(grad.begin(), grad.end(), 0);
//         for (uint i=0; i<7; ++i)
//         {
//             grad[i + g_data->idx*g_data->num_phase_variables] = J(0, i);
//         }
//         grad[7 + g_data->idx*g_data->num_phase_variables] = -1;
//     }
//     return g;
// }

void 
Constraint::ManipulationFrame(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(f_data);
    Eigen::Map<const Eigen::VectorXd> q(x, 13);
    auto kin_conf = kinematics::GeometricJacobian(q, true);
    Eigen::MatrixXd J = kin_conf[0];
    Eigen::MatrixXd FK = kin_conf[1];

    result[0] = FK(0,3) - q(7);
    result[1] = FK(1,3) - q(8);
    result[2] = FK(2,3) - q(9);

    if (grad)
    {
        for (uint i=0; i<7; ++i)
        {
            grad[i] = J(0, i);
            grad[i+n] = J(1,i);
            grad[i+2*n] = J(2,i);
        }
        grad[7] = -1;
        grad[8+n] = -1;
        grad[9+2*n] = -1;
    }
    return;
}