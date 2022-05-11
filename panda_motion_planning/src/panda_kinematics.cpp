#include <panda_motion_planning/panda_kinematics.h>

namespace panda_kinematics
{

Kinematics::Kinematics(ros::NodeHandle *nh)
{
    _sub_franka_state = nh->subscribe("/franka_state_controller/joint_states", 1, &Kinematics::FrankaStateCallback, this);
}


Eigen::Matrix4d Kinematics::DH_matrix(const double a, const double d, 
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

Eigen::Matrix4d Kinematics::ForwardKinematics(std::array<double, 7> joint_position)
{
    auto T1 = DH_matrix(0,          0.333,      0,            joint_position[0]);
    auto T2 = DH_matrix(0,          0,          -M_PI_2,      joint_position[1]);
    auto T3 = DH_matrix(0,          0.316,      M_PI_2,       joint_position[2]);
    auto T4 = DH_matrix(0.0825,     0,          M_PI_2,       joint_position[3]);
    auto T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      joint_position[4]);
    auto T6 = DH_matrix(0,          0,          M_PI_2,       joint_position[5]);
    auto T7 = DH_matrix(0.088,      0,          M_PI_2,       joint_position[6]);
    auto T8 = DH_matrix(0,          0.107,      0,            0);
    return T1*T2*T3*T4*T5*T6*T7*T8;
}

void Kinematics::FrankaStateCallback(const sensor_msgs::JointState &msg)
{
    std::copy(std::begin(msg.position), std::end(msg.position), std::begin(_joint_position));
}


std::array<double, 7> Kinematics::InverseKinematics(std::array<double, 16> O_T_EE_array,
                                                    double q7,
                                                    std::array<double, 7> q_actual_array)
{
    const std::array<double, 7> q_NAN = {{NAN, NAN, NAN, NAN, NAN, NAN, NAN}};
    
    std::array<double, 7> q;
    
    Eigen::Map< Eigen::Matrix<double, 4, 4> > O_T_EE(O_T_EE_array.data());
    
    // constants
    const double d1 = 0.3330;
    const double d3 = 0.3160;
    const double d5 = 0.3840;
    const double d7e = 0.2104;
    const double a4 = 0.0825;
    const double a7 = 0.0880;

    const double LL24 = 0.10666225; // a4^2 + d3^2
    const double LL46 = 0.15426225; // a4^2 + d5^2
    const double L24 = 0.326591870689; // sqrt(LL24)
    const double L46 = 0.392762332715; // sqrt(LL46)
    
    const double thetaH46 = 1.35916951803; // atan(d5/a4);
    const double theta342 = 1.31542071191; // atan(d3/a4);
    const double theta46H = 0.211626808766; // acot(d5/a4);
    
    const std::array<double, 7> q_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, 7> q_max = {{ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973}};
    
    // return NAN if input q7 is out of range
    if (q7 <= q_min[6] || q7 >= q_max[6])
        return q_NAN;
    else
        q[6] = q7;
    
    // FK for getting current case id
    double c1_a = std::cos(q_actual_array[0]); double s1_a = std::sin(q_actual_array[0]);
    double c2_a = std::cos(q_actual_array[1]); double s2_a = std::sin(q_actual_array[1]);
    double c3_a = std::cos(q_actual_array[2]); double s3_a = std::sin(q_actual_array[2]);
    double c4_a = std::cos(q_actual_array[3]); double s4_a = std::sin(q_actual_array[3]);
    double c5_a = std::cos(q_actual_array[4]); double s5_a = std::sin(q_actual_array[4]);
    double c6_a = std::cos(q_actual_array[5]); double s6_a = std::sin(q_actual_array[5]);

    std::array< Eigen::Matrix<double, 4, 4>, 7> As_a;
    As_a[0] <<   c1_a, -s1_a,  0.0,  0.0,    // O1
                 s1_a,  c1_a,  0.0,  0.0,
                  0.0,   0.0,  1.0,   d1,
                  0.0,   0.0,  0.0,  1.0;
    As_a[1] <<   c2_a, -s2_a,  0.0,  0.0,    // O2
                  0.0,   0.0,  1.0,  0.0,
                -s2_a, -c2_a,  0.0,  0.0,
                  0.0,   0.0,  0.0,  1.0;
    As_a[2] <<   c3_a, -s3_a,  0.0,  0.0,    // O3
                  0.0,   0.0, -1.0,  -d3,
                 s3_a,  c3_a,  0.0,  0.0,
                  0.0,   0.0,  0.0,  1.0;
    As_a[3] <<   c4_a, -s4_a,  0.0,   a4,    // O4
                  0.0,   0.0, -1.0,  0.0,
                 s4_a,  c4_a,  0.0,  0.0,
                  0.0,   0.0,  0.0,  1.0;
    As_a[4] <<    1.0,   0.0,  0.0,  -a4,    // H
                  0.0,   1.0,  0.0,  0.0,
                  0.0,   0.0,  1.0,  0.0,
                  0.0,   0.0,  0.0,  1.0;
    As_a[5] <<   c5_a, -s5_a,  0.0,  0.0,    // O5
                  0.0,   0.0,  1.0,   d5,
                -s5_a, -c5_a,  0.0,  0.0,
                  0.0,   0.0,  0.0,  1.0;
    As_a[6] <<   c6_a, -s6_a,  0.0,  0.0,    // O6
                  0.0,   0.0, -1.0,  0.0,
                 s6_a,  c6_a,  0.0,  0.0,
                  0.0,   0.0,  0.0,  1.0;
    std::array< Eigen::Matrix<double, 4, 4>, 7> Ts_a;
    Ts_a[0] = As_a[0];
    for (unsigned int j = 1; j < 7; j++)
        Ts_a[j] = Ts_a[j - 1]*As_a[j];

    // identify q6 case
    Eigen::Vector3d V62_a = Ts_a[1].block<3, 1>(0, 3) - Ts_a[6].block<3, 1>(0, 3);
    Eigen::Vector3d V6H_a = Ts_a[4].block<3, 1>(0, 3) - Ts_a[6].block<3, 1>(0, 3);
    Eigen::Vector3d Z6_a = Ts_a[6].block<3, 1>(0, 2);
    bool is_case6_0 = ((V6H_a.cross(V62_a)).transpose()*Z6_a <= 0);

    // identify q1 case
    bool is_case1_1 = (q_actual_array[1] < 0);
    
    // IK: compute p_6
    Eigen::Matrix3d R_EE = O_T_EE.topLeftCorner<3, 3>();
    Eigen::Vector3d z_EE = O_T_EE.block<3, 1>(0, 2);
    Eigen::Vector3d p_EE = O_T_EE.block<3, 1>(0, 3);
    Eigen::Vector3d p_7 = p_EE - d7e*z_EE;
    
    Eigen::Vector3d x_EE_6;
    x_EE_6 << std::cos(q7 - M_PI_4), -std::sin(q7 - M_PI_4), 0.0;
    Eigen::Vector3d x_6 = R_EE*x_EE_6;
    x_6 /= x_6.norm(); // visibly increases accuracy
    Eigen::Vector3d p_6 = p_7 - a7*x_6;
    
    // IK: compute q4
    Eigen::Vector3d p_2;
    p_2 << 0.0, 0.0, d1;
    Eigen::Vector3d V26 = p_6 - p_2;
    
    double LL26 = V26[0]*V26[0] + V26[1]*V26[1] + V26[2]*V26[2];
    double L26 = std::sqrt(LL26);
    
    if (L24 + L46 < L26 || L24 + L26 < L46 || L26 + L46 < L24)
        return q_NAN;

    
    double theta246 = std::acos((LL24 + LL46 - LL26)/2.0/L24/L46);
    q[3] = theta246 + thetaH46 + theta342 - 2.0*M_PI;
    if (q[3] <= q_min[3] || q[3] >= q_max[3])
        return q_NAN;
    
    // IK: compute q6
    double theta462 = std::acos((LL26 + LL46 - LL24)/2.0/L26/L46);
    double theta26H = theta46H + theta462;
    double D26 = -L26*std::cos(theta26H);
    
    Eigen::Vector3d Z_6 = z_EE.cross(x_6);
    Eigen::Vector3d Y_6 = Z_6.cross(x_6);
    Eigen::Matrix3d R_6;
    R_6.col(0) = x_6;
    R_6.col(1) = Y_6/Y_6.norm();
    R_6.col(2) = Z_6/Z_6.norm();
    Eigen::Vector3d V_6_62 = R_6.transpose()*(-V26);

    double Phi6 = std::atan2(V_6_62[1], V_6_62[0]);
    double Theta6 = std::asin(D26/std::sqrt(V_6_62[0]*V_6_62[0] + V_6_62[1]*V_6_62[1]));
    
    if (is_case6_0)
        q[5] = M_PI - Theta6 - Phi6;
    else
        q[5] = Theta6 - Phi6;
    
    if (q[5] <= q_min[5])
        q[5] += 2.0*M_PI;
    else if (q[5] >= q_max[5])
        q[5] -= 2.0*M_PI;
    
    if (q[5] <= q_min[5] || q[5] >= q_max[5])
        return q_NAN;
    

    // IK: compute q1 & q2
    double thetaP26 = 3.0*M_PI_2 - theta462 - theta246 - theta342;
    double thetaP = M_PI - thetaP26 - theta26H;
    double LP6 = L26*sin(thetaP26)/std::sin(thetaP);
    
    Eigen::Vector3d z_6_5;
    z_6_5 << std::sin(q[5]), std::cos(q[5]), 0.0;
    Eigen::Vector3d z_5 = R_6*z_6_5;
    Eigen::Vector3d V2P = p_6 - LP6*z_5 - p_2;
    
    double L2P = V2P.norm();
    
    if (std::fabs(V2P[2]/L2P) > 0.999)
    {
        q[0] = q_actual_array[0];
        q[1] = 0.0;
    }
    else
    {
        q[0] = std::atan2(V2P[1], V2P[0]);
        q[1] = std::acos(V2P[2]/L2P);
        if (is_case1_1)
        {
            if (q[0] < 0.0)
                q[0] += M_PI;
            else
                q[0] -= M_PI;
            q[1] = -q[1];
        }
    }
    
    if ( q[0] <= q_min[0] || q[0] >= q_max[0]
      || q[1] <= q_min[1] || q[1] >= q_max[1] )
        return q_NAN;
    
    // IK: compute q3
    Eigen::Vector3d z_3 = V2P/V2P.norm();
    Eigen::Vector3d Y_3 = -V26.cross(V2P);
    Eigen::Vector3d y_3 = Y_3/Y_3.norm();
    Eigen::Vector3d x_3 = y_3.cross(z_3);
    Eigen::Matrix3d R_1;
    double c1 = std::cos(q[0]);
    double s1 = std::sin(q[0]);
    R_1 <<   c1,  -s1,  0.0,
             s1,   c1,  0.0,
            0.0,  0.0,  1.0;
    Eigen::Matrix3d R_1_2;
    double c2 = std::cos(q[1]);
    double s2 = std::sin(q[1]);
    R_1_2 <<   c2,  -s2, 0.0,
              0.0,  0.0, 1.0,
              -s2,  -c2, 0.0;
    Eigen::Matrix3d R_2 = R_1*R_1_2;
    Eigen::Vector3d x_2_3 = R_2.transpose()*x_3;
    q[2] = std::atan2(x_2_3[2], x_2_3[0]);
    
    if (q[2] <= q_min[2] || q[2] >= q_max[2])
        return q_NAN;
    
    // IK: compute q5
    Eigen::Vector3d VH4 = p_2 + d3*z_3 + a4*x_3 - p_6 + d5*z_5;
    Eigen::Matrix3d R_5_6;
    double c6 = std::cos(q[5]);
    double s6 = std::sin(q[5]);
    R_5_6 <<   c6,  -s6,  0.0,
              0.0,  0.0, -1.0,
               s6,   c6,  0.0;
    Eigen::Matrix3d R_5 = R_6*R_5_6.transpose();
    Eigen::Vector3d V_5_H4 = R_5.transpose()*VH4;
    
    q[4] = -std::atan2(V_5_H4[1], V_5_H4[0]);
    if (q[4] <= q_min[4] || q[4] >= q_max[4])
        return q_NAN;
    
    return q;
}                                                    



}// namespace panda_kinematics