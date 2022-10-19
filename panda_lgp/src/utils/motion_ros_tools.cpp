#include <panda_lgp/utils/motion_ros_tools.h>


///////////////////////////////////////////////////////////////////////
/// @brief Creates publishers for markers within RViz and for panda joint controller
/// @param nh ROS NodeHandle from the main program
///////////////////////////////////////////////////////////////////////
MotionROSTools::MotionROSTools(ros::NodeHandle &nh)
{
    _marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    _joint_pub = nh.advertise<panda_gazebo_controllers::JointPosition>("/panda/joint_position_goal", 1000);
    _joint_state_sub = nh.subscribe("/joint_states", 1, &MotionROSTools::JointStateCallback, this);
}

///////////////////////////////////////////////////////////////////////
/// @brief Publish desired joints positions to the controller each "dt"
/// @param TODO: dont like to format
///////////////////////////////////////////////////////////////////////
void 
MotionROSTools::ExecuteTrajectory(const std::vector<std::vector<double>> results, const double traj_time)
{
    double dt = traj_time / static_cast<double>(results[0].size());
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
    ROS_INFO("Trajectory command sent...");
}


void 
MotionROSTools::JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (uint idx=0; idx<configuration.q_dim; idx++)
    {
        configuration.q_act[idx] = msg->position[idx];
    }
}


void MotionROSTools::VisualizeTrajectory(const std::vector<std::vector<double>> trajectory, const std_msgs::ColorRGBA &color, const int id)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id;
    line_strip.type = visualization_msgs::Marker::POINTS;
    line_strip.scale.x = 0.01;
    line_strip.scale.y = 0.01;
    // line_strip.color = color;

    for (size_t t=0; t < trajectory.size(); t++)
    {
        Eigen::VectorXd q(7);
        q << trajectory[t][0],
             trajectory[t][1],
             trajectory[t][2],
             trajectory[t][3],
             trajectory[t][4],
             trajectory[t][5],
             trajectory[t][6];

        Eigen::Matrix4d T = kinematics::ForwardKinematics(q, true);
        geometry_msgs::Point p;
        p.x = T(0,3);
        p.y = T(1,3);
        p.z = T(2,3);
        line_strip.points.push_back(p);
        line_strip.colors.push_back(color);
    }
    _marker_pub.publish(line_strip);
    ROS_INFO("Visualizing trajectory...");
}
