#include <panda_komo_ipopt/constraints/add_point_to_point_distance.h>

std::vector<std::vector<double>> AddPointToPointDistanceConstraint::g_jac;


AddPointToPointDistanceConstraint::AddPointToPointDistanceConstraint()
    : AddPointToPointDistanceConstraint("point_to_point_distance_constraint", 20, 0.3)
{
}

AddPointToPointDistanceConstraint::AddPointToPointDistanceConstraint(const std::string& name, const int num_time_slices, const double tolerance)
    : ifopt::ConstraintSet(num_time_slices, name)
{
    _tolerance = tolerance;
    _object_pos << 0.5, 0, 0.1;
}


Eigen::VectorXd AddPointToPointDistanceConstraint::GetValues() const
{
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();
    g_jac.clear();


    for (size_t idx=0; idx<20; idx++)
    {
        autodiff::VectorXreal x_autodiff = Eigen::Map<Eigen::Matrix<double, 7, 1> >(x.data()+idx*7, 7);

        autodiff::real g_autodiff;
        Eigen::VectorXd gradient = autodiff::gradient(ComputePointToPointDistanceConstraint, wrt(x_autodiff), at(x_autodiff, _object_pos), g_autodiff);
        g(idx) = g_autodiff.val();

        // std::vector<double> temp(gradient.data(), gradient.data()+gradient.rows());
        // g_jac.push_back(temp);
        g_jac.push_back(std::vector<double>(gradient.data(), gradient.data()+gradient.rows()));
        // std::cout << temp << std::endl;
        // std::cout << "g(idx) = " << g(idx) << std::endl;


        // Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*7, 7);
        // auto FK_q = Kinematics::ForwardKinematics(q, true);
        // Eigen::VectorXd pos_t(3);
        // pos_t << FK_q(0,3), FK_q(1,3), FK_q(2,3);
        // auto diff_pos = pos_t - _object_pos;
        // // std::cout << "Diff_pos: \n" << diff_pos << std::endl;
        // double l2_norm = sqrt(diff_pos.transpose() * diff_pos);
        // g(idx) = l2_norm;
        // std::cout << "gradient: \n" << gradient.transpose() << std::endl;
    }
    // std::cout << "G = \n" << g << std::endl;
    return g;
}


AddPointToPointDistanceConstraint::VecBound AddPointToPointDistanceConstraint::GetBounds() const
{
    VecBound bounds(GetRows());
    for (auto &b : bounds)
    {
        b = ifopt::Bounds(_tolerance, ifopt::inf);
    }
    return bounds;
}

void AddPointToPointDistanceConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac_block) const
{
    if (var_set == "x") 
    {
        VectorXd x = GetVariables()->GetComponent("x")->GetValues();

        // Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*7, 7);
        // std::cout << "rows: " << jac_block.rows() << "\t cols: " << jac_block.cols() << std::endl;

        // First row

        for (size_t idx=0; idx < 7; idx++)
        {
            jac_block.coeffRef(0, idx+7*0) = g_jac[0][idx];
            jac_block.coeffRef(1, idx+7*1) = g_jac[1][idx];
            jac_block.coeffRef(2, idx+7*2) = g_jac[2][idx];
            jac_block.coeffRef(3, idx+7*3) = g_jac[3][idx];
            jac_block.coeffRef(4, idx+7*4) = g_jac[4][idx];
            jac_block.coeffRef(5, idx+7*5) = g_jac[5][idx];
            jac_block.coeffRef(6, idx+7*6) = g_jac[6][idx];
            jac_block.coeffRef(7, idx+7*7) = g_jac[7][idx];
            jac_block.coeffRef(8, idx+7*8) = g_jac[8][idx];
            jac_block.coeffRef(9, idx+7*9) = g_jac[9][idx];
            jac_block.coeffRef(10, idx+7*10) = g_jac[10][idx];
            jac_block.coeffRef(11, idx+7*11) = g_jac[11][idx];
            jac_block.coeffRef(12, idx+7*12) = g_jac[12][idx];
            jac_block.coeffRef(13, idx+7*13) = g_jac[13][idx];
            jac_block.coeffRef(14, idx+7*14) = g_jac[14][idx];
            jac_block.coeffRef(15, idx+7*15) = g_jac[15][idx];
            jac_block.coeffRef(16, idx+7*16) = g_jac[16][idx];
            jac_block.coeffRef(17, idx+7*17) = g_jac[17][idx];
            jac_block.coeffRef(18, idx+7*18) = g_jac[18][idx];
            jac_block.coeffRef(19, idx+7*19) = g_jac[19][idx];
        }

        // for (size_t row=0; row < 20; row++)
        // {
        //     for (size_t idx=0; idx < 7; idx++)
        //     {
        //         jac_block.coeffRef(row, idx+7*row) = g_jac[row][idx];
        //     }
        // }
    }

}


autodiff::MatrixXreal AddPointToPointDistanceConstraint::DH_matrix(const autodiff::real a, const autodiff::real d, 
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


autodiff::real AddPointToPointDistanceConstraint::ComputePointToPointDistanceConstraint
    (autodiff::VectorXreal q, autodiff::VectorXreal obj_position)
{
    // 1.step: Compute FK
    auto T1 = DH_matrix(0,          0.333,      0,            q(0));
    auto T2 = DH_matrix(0,          0,          -M_PI_2,      q(1));
    auto T3 = DH_matrix(0,          0.316,      M_PI_2,       q(2));
    auto T4 = DH_matrix(0.0825,     0,          M_PI_2,       q(3));
    auto T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      q(4));
    auto T6 = DH_matrix(0,          0,          M_PI_2,       q(5));
    auto T7 = DH_matrix(0.088,      0,          M_PI_2,       q(6));
    auto T8 = DH_matrix(0,          0.210,      0,            -0.785);      // gripper is included
    auto FK = T1*T2*T3*T4*T5*T6*T7*T8;

    autodiff::VectorXreal eef_position(3);
    eef_position << FK(0,3), FK(1,3), FK(2,3);
    auto temp = eef_position-obj_position;
    return sqrt((temp.transpose()*temp)[0]);
}