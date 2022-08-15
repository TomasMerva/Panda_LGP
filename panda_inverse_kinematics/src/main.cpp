#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <panda_gazebo_controllers/JointPosition.h>

#include <panda_inverse_kinematics/inverse_kinematics.h>
#include <panda_inverse_kinematics/utils/motion_planning_tools.h>

#include <gnuplot_module/gnuplot_module.h>

std::vector<double> q_start(7);
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
    std::vector<double> translation{msg->data[0], msg->data[1], msg->data[2]};
    std::vector<double> rotation{msg->data[3], msg->data[4], msg->data[5]};
    InverseKinematics ik;
    ik.AddPositionConstraint(translation);
    ik.AddOrientationConstraint(rotation);
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

    Eigen::VectorXd temp = Eigen::Map<Eigen::VectorXd>(&results[0], results.size());
    auto FK = Kinematics::ForwardKinematics(temp, true);
    Eigen::MatrixXd R = FK.block<3,3>(0,0); 
    auto rpy = Kinematics::RPYFromRotationMatrix(R);
    std::cout << "RPY: \t " << rpy.transpose() << std::endl;

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