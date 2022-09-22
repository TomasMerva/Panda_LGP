#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

#include <panda_gazebo_controllers/JointPosition.h>
#include <panda_lgp/utils/kinematics.h>

class MotionROSTools
{
    public:
        MotionROSTools(ros::NodeHandle &nh);

        // Visualize trajectory
        void VisualizeTrajectory(const std::vector<std::vector<double>> results, bool gripper);

        // Execute trajectory
        void ExecuteTrajectory(const std::vector<std::vector<double>> results, const double traj_time);

    private:
        ros::Publisher _marker_pub;
        ros::Publisher _joint_pub;
};
