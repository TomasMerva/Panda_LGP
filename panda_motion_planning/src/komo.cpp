#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <panda_gazebo_controllers/JointPosition.h>

#include <panda_motion_planning/komo_lib.h>
#include <gnuplot_module/gnuplot_module.h>

#include <panda_motion_planning/constraints/add_point_to_point_distance.h>

std::vector<double> q_start(7);

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    q_start = msg->position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "komo");
    ros::NodeHandle nh;

    std::vector<double> init_pos{-0.4593, 0.3762, -0.1474, -2.0439, 0.0354, 2.4312, 0.1813};    // starting position
    AddPointToPointDistanceConstraint con;
    std::cout << con.GetValues(init_pos) << std::endl;

    // // Arguments from launch file
    // std::string execute_arg, visualize_arg, plot_arg;
    // execute_arg = argv[1];
    // visualize_arg = argv[2];
    // plot_arg = argv[3];

    // // ROS init
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    // ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);

    // const int num_joints = 7;
    // const int num_time_slices = 20;
    // std::vector<double> init_pos{-0.4593, 0.3762, -0.1474, -2.0439, 0.0354, 2.4312, 0.1813};    // starting position

    // // safety break
    // ros::Duration(1).sleep();

    // // KOMO
    // KOMO komo(nh, num_joints, num_time_slices, 2);
    // komo.UpdateStates(q_start, init_pos);
    // // each vector in results is separate joint, each joint consists of num_time_slices elements
    // std::vector<std::vector<double>> results = komo.Optimize();
    // komo.ExecuteTrajectory(results, 5.0);   // init trajectory

    // ros::Duration(3).sleep();   // safety break
    // std::vector<double> goal_2{0.6569, 0.5686, 0.1156, -1.739, -0.055, 2.3598, 1.5583};    // second position
    // komo.UpdateStates(q_start, goal_2);
    // results = komo.Optimize();

    // // Plot results
    // if (plot_arg == "true")
    // {
    //     GnuplotModule plot;
    //     // plot.Subplot_Yranges(komo.GetJointLimits());
    //     std::vector<double> time = linspace(0, 5, num_time_slices);
    //     std::vector<std::string> labels{"q1", "q2", "q3", "q4", "q5", "q6", "q7"};
    //     plot.SubplotData(time, results, labels);
    // }


    // while(true)
    // {
    //     if (visualize_arg == "true")
    //     {
    //         komo.VisualizeTrajectory(results, true);
    //     }

    //     std::string answer;
    //     std::cout << "Do you want to execute?";
    //     std::cin >> answer;

    //     // Execute trajectory
    //     if (answer == "y")
    //     {
    //         komo.ExecuteTrajectory(results, 5.0);
    //     }
    //     ros::Duration(0.1).sleep();
    // }
    
    return 0;
}

//[-0.4593389643071841, 0.3762131523805783, -0.14746962768825078, -2.0439627184936606, 0.03542332781547053, 2.4312440745296495, 0.18137867106475802]
//[0.6569053790824997, 0.5686354077824252, 0.11562161775950397, -1.7390710728511003, -0.0550287460358696, 2.3598245597471994, 1.5583802506773168]
