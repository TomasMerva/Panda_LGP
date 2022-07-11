#include <ros/ros.h>
#include <panda_motion_planning/panda_ik_analytic.h>
#include <iostream>
#include <chrono>

#include <panda_motion_planning/panda_kinematics.h>
//#include <panda_gazebo_controllers/JointPosition.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_motion_planning");
    ros::NodeHandle nh;
/*
    ros::Publisher pub = nh.advertise<panda_gazebo_controllers::JointPosition>("/panda/joint_position_goal", 1);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    panda_kinematics::Kinematics panda(&nh);
    ros::Duration(0.1).sleep();

    std::array<double, 16> O_T_EE_array = {1, 0, 0, 0,
                                           0, -1, 0, 0,
                                           0, 0, -1, 0,
                                           0.5, 0, 0.05, 1};


    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    auto t1 = high_resolution_clock::now();

    auto result = panda.InverseKinematics(O_T_EE_array, panda._joint_position[6], panda._joint_position);
    // auto result = franka_IK_EE_CC(O_T_EE_array, panda._joint_position[6], panda._joint_position);


    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << ms_double.count() << "ms\n";

    panda_gazebo_controllers::JointPosition msg;

    std::copy(std::begin(result), std::end(result), std::begin(msg.joint_position));
    pub.publish(msg);
*/
    ros::waitForShutdown();
    return 0;
}
