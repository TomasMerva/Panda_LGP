#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace panda_kinematics{

class Kinematics
{
    public:
        Kinematics();

        // Forward Kinematics
        Eigen::Matrix4d DH_matrix(const double a, const double d, 
                                  const double alpha, const double theta);
        Eigen::Matrix4d ForwardKinematics(std::array<double, 7> joint_position, bool gripper_enable);

        // Inverse Kinematics
        std::array<double, 7> InverseKinematics(std::array<double, 16> O_T_EE_array,
                                                double q7,
                                                std::array<double, 7> q_actual_array);

        struct position
        {
            double x;
            double y;
            double z;
        };
        

    private:

};

} //namespace panda_kinematics

