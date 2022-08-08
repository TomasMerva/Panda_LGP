#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <panda_gazebo_controllers/JointPosition.h>

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

    // Arguments from launch file
    std::string execute_arg, visualize_arg, plot_arg;
    execute_arg = argv[1];
    visualize_arg = argv[2];
    plot_arg = argv[3];

    // ROS init
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);

    const int num_joints = 7;
    const int num_time_slices = 20;
    std::vector<double> q_goal{0.096, 0.308, 0.646, -2.249, 0.06, 2.53, 2.019}; 

    // safety break
    ros::Duration(0.5).sleep();

    // KOMO
    KOMO komo(nh, num_joints, num_time_slices, 2);
    komo.UpdateStates(q_start, q_goal);
    komo.AddConstraint(FS_PointToPointDistance);    // Add which timesteps it constraints + Add phases
    komo.AddConstraint(FS_FixedOrientationAxis);

    // each vector in results is separate joint, each joint consists of num_time_slices elements
    std::vector<std::vector<double>> results = komo.Optimize();

    // Plot results
    if (plot_arg == "true")
    {
        GnuplotModule plot;
        // plot.Subplot_Yranges(komo.GetJointLimits());
        std::vector<double> time = linspace(0, 5, num_time_slices);
        std::vector<std::string> labels{"q1", "q2", "q3", "q4", "q5", "q6", "q7"};
        plot.SubplotData(time, results, labels);
    }

    ros::Duration(0.5).sleep();
    while(true)
    {
        if (visualize_arg == "true")
        {
            komo.VisualizeTrajectory(results, true);
        }

        std::string answer;
        std::cout << "Do you want to execute?";
        std::cin >> answer;

        // Execute trajectory
        if (answer == "y")
        {
            komo.ExecuteTrajectory(results, 5.0);
        }
        else
        {
            return 0;
        }
        ros::Duration(0.1).sleep();
    }
    
    ros::waitForShutdown();
    return 0;
}
