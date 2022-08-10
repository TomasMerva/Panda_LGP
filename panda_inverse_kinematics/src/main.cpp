#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <panda_gazebo_controllers/JointPosition.h>

#include <panda_inverse_kinematics/inverse_kinematics.h>
#include <panda_inverse_kinematics/utils/motion_planning_tools.h>

#include <gnuplot_module/gnuplot_module.h>

std::vector<double> q_start(7);
std::vector<double> pose_goal(6);
ros::Publisher joint_pub;

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (int idx=0; idx<7; idx++)
    {
        q_start[idx] = msg->position[idx];
    }
}

void SlidersCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    memcpy(&pose_goal[0], &msg->data[0], sizeof(msg->data));
    std::vector<double> translation{pose_goal[0], pose_goal[1], pose_goal[2]};
    InverseKinematics ik;
    ik.AddPositionConstraint(translation);
    // ik.AddConstraint(FS_PositionConstraint);
    //ik.AddOrientationConstraint()
    ik.SetInitialGuess(q_start);
    auto results = ik.Solve();
    ROS_INFO("IK Success...");
    for (auto q : results)
    {
        std::cout << q << " ";
    }
    std::cout << "\n\n" << std::endl;
    panda_gazebo_controllers::JointPosition joint_msg;
    // memcpy(&msg.joint_position[0], &results[0], sizeof(results));
    joint_msg.joint_position[0] = results[0];
    joint_msg.joint_position[1] = results[1];
    joint_msg.joint_position[2] = results[2];
    joint_msg.joint_position[3] = results[3];
    joint_msg.joint_position[4] = results[4];
    joint_msg.joint_position[5] = results[5];
    joint_msg.joint_position[6] = results[6];
    joint_pub.publish(joint_msg);  
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics");
    ros::NodeHandle nh;

    // ROS init
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);
    ros::Subscriber sliders_sub = nh.subscribe("/PoseSlidersCallback", 1, SlidersCallback);
    joint_pub = nh.advertise<panda_gazebo_controllers::JointPosition>("/panda/joint_position_goal", 1000);

    // safety break
    ros::Duration(0.5).sleep();


    ros::waitForShutdown();
    return 0;
}