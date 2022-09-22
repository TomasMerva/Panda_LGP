#include <panda_lgp/utils/kinematics.h>

namespace kinematics
{

///////////////////////////////////////////////////////////////////////
/// @brief Computes Denavit-Hartenberg matrix for the purpose of forward kinematics
/// @param a -> distance between the z_i and z_{i-1} axes
/// @param d -> signed distance between the x_i and x_{i-1} axes
/// @param alpha -> angle between the z_i and z_{i-1} axes
/// @param theta -> angle between the x_i and x_{i-1} axes (rotation about the z_i axis)
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d DH_matrix(const double a, const double d, 
                          const double alpha, const double theta)
{
    Eigen::MatrixXd T(4,4); 
    double sin_alpha = sin(alpha), cos_alpha = cos(alpha);
    double sin_theta = sin(theta), cos_theta = cos(theta);
    T << cos_theta,              -sin_theta,                   0,                a,
         sin_theta*cos_alpha,    cos_theta*cos_alpha,          -sin_alpha,       -sin_alpha*d,
         sin_theta*sin_alpha,    cos_theta*sin_alpha,          cos_alpha,        cos_alpha*d,
         0,                      0,                            0,                1;
    return T;
}                          

///////////////////////////////////////////////////////////////////////
/// @brief Computes Forward kinematics for Franka Emika by multiplying DH matrices
/// @param q -> joint angles
/// @param gripper_enable -> is gripper used or not
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d ForwardKinematics(Eigen::VectorXd q, bool gripper_enable)
{
    Eigen::Matrix4d T1, T2, T3, T4, T5, T6, T7, T8;
    if (gripper_enable)
    {
        T1 = DH_matrix(0,          0.333,      0,            q(0));
        T2 = DH_matrix(0,          0,          -M_PI_2,      q(1));
        T3 = DH_matrix(0,          0.316,      M_PI_2,       q(2));
        T4 = DH_matrix(0.0825,     0,          M_PI_2,       q(3));
        T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      q(4));
        T6 = DH_matrix(0,          0,          M_PI_2,       q(5));
        T7 = DH_matrix(0.088,      0,          M_PI_2,       q(6));
        T8 = DH_matrix(0,          0.210,      0,            -0.785); // Flange and gripper frame is added to
    }
    else
    {
        T1 = DH_matrix(0,          0.333,      0,            q(0));
        T2 = DH_matrix(0,          0,          -M_PI_2,      q(1));
        T3 = DH_matrix(0,          0.316,      M_PI_2,       q(2));
        T4 = DH_matrix(0.0825,     0,          M_PI_2,       q(3));
        T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      q(4));
        T6 = DH_matrix(0,          0,          M_PI_2,       q(5));
        T7 = DH_matrix(0.088,      0,          M_PI_2,       q(6));
        T8 = DH_matrix(0,          0.107,      0,            0);
    }    
    return T1*T2*T3*T4*T5*T6*T7*T8; 
}


///////////////////////////////////////////////////////////////////////
/// @brief Computes Geometric Jacobian for Franka Emika (7x revolute joints)
/// @param q -> joint angles
/// @param gripper_enable -> is gripper used or not
///////////////////////////////////////////////////////////////////////
Eigen::MatrixXd GeometricJacobian(Eigen::VectorXd q, bool gripper_enable)
{
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,7);
    Eigen::Matrix4d T1, T2, T3, T4, T5, T6, T7, T8;
    if (gripper_enable)
    {
        T1 = DH_matrix(0,          0.333,      0,            q(0));
        T2 = DH_matrix(0,          0,          -M_PI_2,      q(1));
        T3 = DH_matrix(0,          0.316,      M_PI_2,       q(2));
        T4 = DH_matrix(0.0825,     0,          M_PI_2,       q(3));
        T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      q(4));
        T6 = DH_matrix(0,          0,          M_PI_2,       q(5));
        T7 = DH_matrix(0.088,      0,          M_PI_2,       q(6));
        T8 = DH_matrix(0,          0.210,      0,            -0.785); // Flange and gripper frame is added to
    }
    else
    {
        T1 = DH_matrix(0,          0.333,      0,            q(0));
        T2 = DH_matrix(0,          0,          -M_PI_2,      q(1));
        T3 = DH_matrix(0,          0.316,      M_PI_2,       q(2));
        T4 = DH_matrix(0.0825,     0,          M_PI_2,       q(3));
        T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      q(4));
        T6 = DH_matrix(0,          0,          M_PI_2,       q(5));
        T7 = DH_matrix(0.088,      0,          M_PI_2,       q(6));
        T8 = DH_matrix(0,          0.107,      0,            0);
    }
    std::vector<Eigen::Matrix4d> transformations{T1, T2, T3, T4, T5, T6, T7*T8};
    auto X_EEF = T1*T2*T3*T4*T5*T6*T7*T8;
    auto p_EEF = X_EEF.col(3);
    
    auto T_temp = transformations[0];
    for (int i=0; i<7; ++i)
    {
        auto z_i = T_temp.col(2);
        auto p_i = T_temp.col(3);
        auto delta = p_EEF - p_i;
        J(0,i) = z_i(1)*delta(2) - z_i(2)*delta(1);
        J(1,i) = -(z_i(0)*delta(2) - z_i(2)*delta(0));
        J(2,i) = z_i(0)*delta(1) - z_i(1)*delta(0);
        J(3,i) = z_i(0);
        J(4,i) = z_i(1);
        J(5,i) = z_i(2);
        
        if(i<6) T_temp = T_temp * transformations[i+1];
    }

    return J;
}

} // namespace