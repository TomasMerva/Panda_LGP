#include <panda_lgp/constraints/constraints.h>



void Constraint::AxisInRegion(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(f_data);
    Eigen::VectorXd test(g_data->num_phase_variables);
    for (int i= g_data->idx; i<g_data->idx*g_data->num_phase_variables; i++)
    {
        test(i-g_data->idx) = x[i];
    }
    // test << x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12];
    Eigen::Map<const Eigen::VectorXd> q(x+ g_data->idx*g_data->num_phase_variables, g_data->num_phase_variables);

    std::cout << test.transpose() << "\n\n";
    std::cout << q.transpose() << std::endl;

    auto kin_conf = kinematics::GeometricJacobian(q, true);
    Eigen::MatrixXd J = kin_conf[0];
    Eigen::MatrixXd FK = kin_conf[1];

    result[0] = - FK(1,3) + g_data->region[0];
    result[1] = FK(1,3) - g_data->region[1];


    if (grad)
    {
        for (uint i=0; i<7; ++i)
        {
            grad[i+ g_data->idx*g_data->num_phase_variables] = -J(1, i);
            grad[i + g_data->idx*g_data->num_phase_variables +n] = J(1, i);
        }
    }
}


// // Euclidean distance between [y] and middle of the region
// double Constraint::AxisInRegion(const std::vector<double> &x, std::vector<double> &grad, void *data)
// {
//     // Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(data);    
//     // double middle_region = g_data->region[0] - (g_data->region[0] - g_data->region[1])/2.0;
//     // double diff = x[8 + g_data->idx*g_data->num_phase_variables] - middle_region;
//     // double g = std::sqrt( std::pow(diff,2) );
//     // if (!grad.empty())
//     // {
//     //     // std::fill(grad.begin(), grad.end(), 0);
//     //     grad[8 + g_data->idx*g_data->num_phase_variables] = diff/g;
//     // }
//     // std::cout << g << "    "; 
//     // return g;

//     double region_limit;
//     Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(data);  
//     if (g_data->region[0] == -0.5)  //grey region
//     {
//         double g = x[8 + g_data->idx*g_data->num_phase_variables] - g_data->region[1];
//         if (!grad.empty())
//         {
//             // std::fill(grad.begin(), grad.end(), 0);
//             grad[8 + g_data->idx*g_data->num_phase_variables] = 1;
//         }
//         return g;
//     }
//     else    // red region
//     {
//         double g = - x[8 + g_data->idx*g_data->num_phase_variables] + g_data->region[0];
//         if (!grad.empty())
//         {
//             grad[8 + g_data->idx*g_data->num_phase_variables] = -1;
//         }
//         return g;
//     }
// }

