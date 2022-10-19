#include <panda_lgp/constraints/constraints.h>

void Constraint::Zaxis(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(f_data);

    Eigen::VectorXd g_t(m);
    Eigen::MatrixXd J_matrix(n, m);
    for (uint t=0; t < m; ++t)
    {
        Eigen::Map<const Eigen::VectorXd> temp_q(x+t*g_data->num_phase_variables, g_data->num_phase_variables);
        auto kin_conf = kinematics::GeometricJacobian(temp_q, true);
        Eigen::MatrixXd J = kin_conf[0];
        Eigen::MatrixXd FK = kin_conf[1];
        
        g_t(t) = -FK(2,3) + 0.025;
        J_matrix.block<7, 1>(t*7, t) = J.row(2);    //change 7
    }
    Eigen::VectorXd g_t_minus1(m);
    Eigen::VectorXd g_t_minus2(m);
    g_t_minus1 << g_t.head(1), g_t.head(m-1);
    g_t_minus2 << g_t_minus1.head(1), g_t_minus1.head(m-1);


    Eigen::Map<Eigen::VectorXd> J_t(J_matrix.data(), m*n);
    Eigen::VectorXd J_t_minus1(J_t.size());
    Eigen::VectorXd J_t_minus2(J_t.size());
    J_t_minus1 << J_t.head(n), J_t.head(J_t.size()-n);
    J_t_minus2 << J_t_minus1.head(n), J_t_minus1.head(J_t.size()-n);
    
    Eigen::VectorXd temp_grad = -J_t -J_t_minus1 - J_t_minus2;
    // std::cout << "g1_grad:----\n";
    // std::cout << temp_grad.block<28, 1>(0,0).transpose() << "\n";
    // std::cout << "g2_grad:----\n";
    // std::cout << temp_grad.block<28, 1>(28,0).transpose() << "\n";
    // std::cout << "g3_grad:----\n";
    // std::cout << temp_grad.block<28, 1>(56,0).transpose() << "\n";
    // std::cout << "g4_grad:----\n";
    // std::cout << temp_grad.block<28, 1>(84,0).transpose() << "\n";


    for (uint t=0; t<m; ++t)
    {
        result[t] = (-g_t(t) + 0.025) + (-g_t_minus1(t) + 0.025) + (-g_t_minus2(t) + 0.025);
        if (grad)
        {
            for (uint i=0; i<temp_grad.size(); ++i)
            {
                grad[i] = -temp_grad(i);
            }
        }
    }
}

// void Constraint::Zaxis(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
// {
//     Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(f_data);
//     if(grad)
//     {
//         std::fill(grad, grad+n*m, 0);
//     }
//     for (uint t=0; t < m; ++t)
//     {
//         Eigen::Map<const Eigen::VectorXd> temp_q(x+t*g_data->num_phase_variables, g_data->num_phase_variables);
//         auto kin_conf = kinematics::GeometricJacobian(temp_q, true);
//         Eigen::MatrixXd J = kin_conf[0];
//         Eigen::MatrixXd FK = kin_conf[1];
        
//         result[t] = -FK(2,3) + 0.025;
        
//         if (grad)
//         {
//             for (uint i=0; i<g_data->num_phase_variables; ++i)
//             {
//                 grad[i+t*n+t*g_data->num_phase_variables] = -J(2, i);
//             }
//         }
//     }
// }