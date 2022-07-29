#include <panda_motion_planning/constraints/add_point_to_point_distance.h>

double Constraint::AddPointToPointDistanceConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    AddPointToPointDistanceData *d = reinterpret_cast<AddPointToPointDistanceData*>(data);

    int num_variables = 7;
    int idx = d->idx; 
    Eigen::VectorXd obj_pos(3);
    obj_pos << d->obj_pos_x, d->obj_pos_y, d->obj_pos_z;
    double tolerance = d->tolerance;

    autodiff::VectorXreal q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*num_variables, num_variables);
    autodiff::real g_autodiff;
    Eigen::VectorXd gradient = autodiff::gradient(ConstraintTools::ComputePointToPointDistanceConstraint, wrt(q), at(q, obj_pos), g_autodiff);

    if (!grad.empty())
    {
        std::fill(grad.begin(), grad.end(), 0);
        for (size_t i=0; i<7; i++)
        {
            grad[i + 7*idx] = gradient[i];
        }
        // for (auto grad_el : grad)
        // {
        //     std::cout << grad_el << " ";
        // }
        // std::cout << std::endl;
        // std::cout << std::endl;
    }
    // double g = - g_autodiff.val() + tolerance;
    return g_autodiff.val();
}


autodiff::MatrixXreal ConstraintTools::DH_matrix(const autodiff::real a, const autodiff::real d, 
                                                 const autodiff::real alpha, const autodiff::real theta)
{
    autodiff::MatrixXreal T(4,4);
    autodiff::real sin_alpha = sin(alpha), cos_alpha = cos(alpha);
    autodiff::real sin_theta = sin(theta), cos_theta = cos(theta);
    T << cos_theta,              -sin_theta,                   0,                a,
         sin_theta*cos_alpha,    cos_theta*cos_alpha,          -sin_alpha,       -sin_alpha*d,
         sin_theta*sin_alpha,    cos_theta*sin_alpha,          cos_alpha,        cos_alpha*d,
         0,                      0,                            0,                1;
    return T;
}                                         

autodiff::real ConstraintTools::ComputePointToPointDistanceConstraint(autodiff::VectorXreal q, autodiff::VectorXreal obj_position)
{
    // 1.step: Compute FK
    auto T1 = ConstraintTools::DH_matrix(0,          0.333,      0,            q(0));
    auto T2 = ConstraintTools::DH_matrix(0,          0,          -M_PI_2,      q(1));
    auto T3 = ConstraintTools::DH_matrix(0,          0.316,      M_PI_2,       q(2));
    auto T4 = ConstraintTools::DH_matrix(0.0825,     0,          M_PI_2,       q(3));
    auto T5 = ConstraintTools::DH_matrix(-0.0825,    0.384,      -M_PI_2,      q(4));
    auto T6 = ConstraintTools::DH_matrix(0,          0,          M_PI_2,       q(5));
    auto T7 = ConstraintTools::DH_matrix(0.088,      0,          M_PI_2,       q(6));
    auto T8 = ConstraintTools::DH_matrix(0,          0.210,      0,            -0.785);      // gripper is included
    auto FK = T1*T2*T3*T4*T5*T6*T7*T8;

    // 2.step: Create a position vector from FK
    autodiff::VectorXreal eef_position(3);
    eef_position << FK(0,3), FK(1,3), FK(2,3);

    // 3.step: Compute Euclidean distance between EEF and object position
    auto temp = eef_position-obj_position;
    return - sqrt((temp.transpose()*temp)[0]) + 0.3;
}