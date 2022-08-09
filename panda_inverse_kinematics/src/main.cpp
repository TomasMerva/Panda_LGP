#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <panda_gazebo_controllers/JointPosition.h>

#include <panda_inverse_kinematics/inverse_kinematics.h>

#include <gnuplot_module/gnuplot_module.h>

std::vector<double> q_start(7);
std::vector<double> pose_goal(6);

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    q_start = msg->position;
}

void SlidersCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    memcpy(&pose_goal[0], &msg->data[0], sizeof(msg->data));
    for (auto q : pose_goal)
    {
        std::cout << q << " ";
    }
    std::cout << std::endl;

    InverseKinematics ik;
    //ik.AddPositionConstraint()
    //ik.AddOrientationConstraint()
    //results = ik.Solve()
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

    // safety break
    ros::Duration(0.5).sleep();

    ros::waitForShutdown();
    return 0;
}