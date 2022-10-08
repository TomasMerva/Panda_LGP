#include <panda_lgp/constraints/constraints.h>

double Constraint::ManipulationFrame(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    Constraint::ConstraintData *g_data = reinterpret_cast<Constraint::ConstraintData*>(data);


    uint num_phases = x.size() / g_data->num_phase_variables;

    double g = 0.0;
    uint idx=0;
    for (uint i=0; i < num_phases; ++i)
    {
        Eigen::Map<const Eigen::VectorXd> q(x.data()+i*g_data->num_phase_variables, g_data->num_phase_variables);
        auto kin_conf = kinematics::GeometricJacobian(q, true);
        Eigen::MatrixXd J = kin_conf[0];
        Eigen::MatrixXd FK = kin_conf[1];
        std::vector<double> RPY = kinematics::ConvertToRPY(FK);
        Eigen::VectorXd g_features(1);
        g_features << FK(0,3) - q(7);
                    //   FK(1,3) - q(8), 
                    //   FK(2,3) - q(9);
                    //   
                    //   RPY[0] - q(10),
                    //   RPY[1] - q(11),
                    //   RPY[2] - q(12);
        g += g_features.sum();
        std::cout << g << std::endl;

        // derivative
        if (!grad.empty())
        {
            for (uint col=0; col<7; ++col)
            {
                grad[idx] = J(0, col);
                idx++;
            }
            
        }
    }
    
    return g;
}