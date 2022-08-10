#include <panda_inverse_kinematics/utils/motion_planning_tools.h>

MotionPlanningTools::MotionPlanningTools(ros::NodeHandle &nh)
{
    _marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    _joint_pub = nh.advertise<panda_gazebo_controllers::JointPosition>("/panda/joint_position_goal", 1000);
}


///////////////////////////////////////////////////////////////////////
/// \brief MotionPlanningTools::DH_matrix
/// \param
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d MotionPlanningTools::DH_matrix(const double a, const double d, 
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
/// \brief MotionPlanningTools::ForwardKinematics
/// \param
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d MotionPlanningTools::ForwardKinematics(std::array<double, 7> joint_position, bool gripper_enable)
{
    if (gripper_enable)
    {
        auto T1 = DH_matrix(0,          0.333,      0,            joint_position[0]);
        auto T2 = DH_matrix(0,          0,          -M_PI_2,      joint_position[1]);
        auto T3 = DH_matrix(0,          0.316,      M_PI_2,       joint_position[2]);
        auto T4 = DH_matrix(0.0825,     0,          M_PI_2,       joint_position[3]);
        auto T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      joint_position[4]);
        auto T6 = DH_matrix(0,          0,          M_PI_2,       joint_position[5]);
        auto T7 = DH_matrix(0.088,      0,          M_PI_2,       joint_position[6]);
        auto T8 = DH_matrix(0,          0.107,      0,            0);
        auto T9 = DH_matrix(0,          0.103,      0,            -0.785);
        return T1*T2*T3*T4*T5*T6*T7*T8*T9; 
    }
    else
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
}

void MotionPlanningTools::SetReferenceTrajectory(const std::vector<std::vector<double>> results, const double traj_time)
{
    _trajectory = results;
    _traj_time = traj_time;
}



///////////////////////////////////////////////////////////////////////
/// \brief MotionPlanningTools::VisualizeTrajectory
/// \param
///////////////////////////////////////////////////////////////////////
void MotionPlanningTools::VisualizeTrajectory(const std::vector<std::vector<double>> results, bool gripper)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::POINTS;
    line_strip.scale.x = 0.01;
    line_strip.scale.y = 0.01;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    for (size_t t=0; t < results[0].size(); t++)
    {
        std::array<double, 7> timestep_joints{results[0][t],
                                                    results[1][t],
                                                    results[2][t],
                                                    results[3][t],
                                                    results[4][t],
                                                    results[5][t],
                                                    results[6][t] };
        Eigen::Matrix4d T = ForwardKinematics(timestep_joints, true);
        geometry_msgs::Point p;
        p.x = T(0,3);
        p.y = T(1,3);
        p.z = T(2,3);
        line_strip.points.push_back(p);
    }
    _marker_pub.publish(line_strip);
    ROS_INFO("Visualizing trajectory...");
}

///////////////////////////////////////////////////////////////////////
/// \brief MotionPlanningTools::ExecuteTrajectory
/// \param
///////////////////////////////////////////////////////////////////////
void MotionPlanningTools::ExecuteTrajectory(const std::vector<std::vector<double>> results, const double traj_time)
{
    double dt = 5.0 / static_cast<double>(results[0].size());
    for (int i=0; i<results[0].size(); i++)
    {
        panda_gazebo_controllers::JointPosition msg;
        msg.joint_position[0] = results[0][i];
        msg.joint_position[1] = results[1][i];
        msg.joint_position[2] = results[2][i];
        msg.joint_position[3] = results[3][i];
        msg.joint_position[4] = results[4][i];
        msg.joint_position[5] = results[5][i];
        msg.joint_position[6] = results[6][i];
        
        _joint_pub.publish(msg);     
        ros::Duration(dt).sleep();
    }
    ROS_INFO("Trajectory execution completed...");
}


