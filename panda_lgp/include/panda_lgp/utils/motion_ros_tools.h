#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>

#include <panda_gazebo_controllers/JointPosition.h>
#include <panda_lgp/utils/kinematics.h>

class MotionROSTools
{
    public:
        MotionROSTools(ros::NodeHandle &nh);

        // Execute trajectory
        void ExecuteTrajectory(const std::vector<std::vector<double>> results, const double traj_time);

        // Subscribe robot state
        void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
        kinematics::Configuration configuration;

        // Visualize Trajectory
        void VisualizeTrajectory(const std::vector<std::vector<double>> trajectory, const std_msgs::ColorRGBA &color, const int id);


    private:
        ros::Publisher _marker_pub;
        ros::Publisher _joint_pub;
        ros::Subscriber _joint_state_sub;
};
