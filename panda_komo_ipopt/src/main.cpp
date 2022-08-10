#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <panda_komo_ipopt/komo_lib.h>
#include <gnuplot_module/gnuplot_module.h>


std::vector<double> q_start(7);

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (int idx=0; idx<7; idx++)
    {
        q_start[idx] = msg->position[idx];
    }
}

template<typename T> std::vector<double> linspace(T start_in, T end_in, int num_in);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "komo_ipopt");
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
    
    // std::vector<double> q_start{-0.7, 0.3762, -0.1474, -2.0439, 0.0354, 2.4312, 0.1813};    // starting position
    // std::vector<double> q_goal{0.6569, 0.5686, 0.1156, -1.739, -0.055, 2.3598, 1.5583};    // second position
    std::vector<double> q_goal{0.096, 0.308, 0.646, -2.249, 0.06, 2.53, 2.019};    // second position

    const int num_joints = 7;
    const int num_time_slices = 20;
    ros::Duration(0.2).sleep();

    KOMO komo(nh, num_joints, num_time_slices);
    komo.UpdateStates(q_start, q_goal);
    // komo.Optimize();
    std::vector<std::vector<double>> results = komo.Optimize();


    ros::Duration(2).sleep();
    // Plot results
    if (plot_arg == "true")
    {
        GnuplotModule plot;
        // plot.Subplot_Yranges(komo.GetJointLimits());
        std::vector<double> time = linspace(0, 5, num_time_slices);
        std::vector<std::string> labels{"q1", "q2", "q3", "q4", "q5", "q6", "q7"};
        plot.SubplotData(time, results, labels);
    }

    while (true)
    {
        if(visualize_arg == "true")
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

    return 0;
}





template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}