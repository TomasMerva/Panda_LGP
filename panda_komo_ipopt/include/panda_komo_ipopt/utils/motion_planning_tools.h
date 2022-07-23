#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>


#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <panda_gazebo_controllers/JointPosition.h>


class MotionPlanningTools
{
    public:
        MotionPlanningTools(ros::NodeHandle &nh);

        // Forward Kinematics
        Eigen::Matrix4d DH_matrix(const double a, const double d, 
                                  const double alpha, const double theta);
        Eigen::Matrix4d ForwardKinematics(std::array<double, 7> joint_position, bool gripper_enable);

        // Visualize trajectory
        void VisualizeTrajectory(const std::vector<std::vector<double>> results, bool gripper);

        // Execute trajectory
        void ExecuteTrajectory(const std::vector<std::vector<double>> results, const double traj_time);

        void SetReferenceTrajectory(const std::vector<std::vector<double>> results, const double traj_time);

        enum MarkerType
        {
            POINTS = 0,
            LINE_STRIP = 1
        };
        

    private:
        ros::Publisher _marker_pub;
        ros::Publisher _joint_pub;
        ros::Subscriber _joint_state_sub;

        std::vector<std::vector<double>> _trajectory;
        double _traj_time;
};

