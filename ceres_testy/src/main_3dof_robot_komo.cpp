#include <ros/ros.h>
#include "ceres_testy/solver.h"
#include "ceres_testy/visualize_rviz.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotCeresKomo");
    ros::NodeHandle nh;

    // ROS init
    ros::AsyncSpinner spinner(1);
    spinner.start();


    VisualizeRviz vis(nh);

    Solver solver(3, 10);
    solver.q_start = std::vector<double>{0.05, 0.0, 0.5};
    solver.q_goal = std::vector<double>{2.0, 0.0, 0.5};
    solver.Solve();

    // // Print results
    // std::cout << "---Results---\n";
    // for (auto xi : solver.x)
    // {
    //     for (uint i=0; i<3; ++i)
    //     {
    //         std::cout << xi[i] << "    ";
    //     }
    //     std::cout << "\n";
    // }

    ros::Duration(1).sleep();
    vis.VisualizeObstacle(std::vector<double>{1, 0, 0.5});
    ros::Duration(0.1).sleep();
    vis.VisualizeTrajectory(solver.x);


    ros::waitForShutdown();
    return 0;
}

