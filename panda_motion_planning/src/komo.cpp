#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <panda_gazebo_controllers/JointPosition.h>

#include <nlopt.hpp>
#include <math.h>

#include <panda_motion_planning/komo_lib.h>
#include <gnuplot_module/gnuplot_module.h>

std::vector<double> q_start(7);

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    q_start = msg->position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "komo");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher joint_pub = nh.advertise<panda_gazebo_controllers::JointPosition>("/panda/joint_position_goal", 1);
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);


    const int num_joints = 7;
    const int num_time_slices = 20;
    // std::vector<double> q_start{0.0, -0.7856, -1.4014, -2.3559, -1.1646, 1.5717, 0.7853};
    std::vector<double> q_goal{-0.2919, 0.785398163, 0, -2.35619449, 0.,1.57079632679, 0.785398163397};

    ros::Duration(1).sleep();

    // KOMO
    KOMO komo(num_joints, num_time_slices, 2);
    komo.UpdateStates(q_start, q_goal);
    std::vector<std::vector<double>> results = komo.Optimize();

    // Visualize results
    // GnuplotModule plot;
    // std::vector<double> time = linspace(0, 5, num_time_slices);
    // std::vector<std::string> labels{"q1", "q2", "q3", "q4", "q5", "q6", "q7"};
    // plot.SubplotData(time, results, labels);
    
        // Visualize results
    GnuplotModule plot;
    std::vector<double> time = linspace(0, 5, num_time_slices);
    std::vector<std::string> labels{"q1", "q2", "q3", "q4", "q5", "q6", "q7"};
    plot.SubplotData(time, results, labels);

    double dt = 5.0 / static_cast<double>(num_time_slices);
    for (int i=0; i<num_time_slices; i++)
    {
        panda_gazebo_controllers::JointPosition msg;
        std::cout << "Publishing time step: " << i << "\n";
        std::cout << sizeof(msg.joint_position) << std::endl;
        msg.joint_position[0] = results[i][0];
        msg.joint_position[1] = results[i][1];
        msg.joint_position[2] = results[i][2];
        msg.joint_position[3] = results[i][3];
        msg.joint_position[4] = results[i][4];
        msg.joint_position[5] = results[i][5];
        msg.joint_position[6] = results[i][6];

        joint_pub.publish(msg);
        ros::Duration(dt).sleep();
    }



    return 0;
}