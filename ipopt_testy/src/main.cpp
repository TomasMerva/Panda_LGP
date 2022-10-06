#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gnuplot_module/gnuplot_module.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

std::vector<double> q_start(7);

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (int idx=0; idx<7; idx++)
    {
        q_start[idx] = msg->position[idx];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "komo_ipopt");
    ros::NodeHandle nh;

    // ROS init
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);

    const int num_phase_variables = 7;
    const int num_phase = 2;
    ros::Duration(0.5).sleep();
    

    // 1. define a problem
    ifopt::Problem nlp;
    // nlp.AddVariableSet
    // nlp.AddCostSet
    // nlp.AddConstraintSet
    nlp.PrintCurrent();

    // 2. Choose Solver
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("jacobian_approximation", "exact");
    // ipopt.SetOption("jacobian_approximation", "finite-difference-values");
    ipopt.SetOption("hessian_approximation", "limited-memory");
    ipopt.SetOption("max_iter", 10000);
    ipopt.SetOption("tol", 1e-8);
    ipopt.SetOption("acceptable_tol", 1e-8);
    // ipopt.SetOption("nlp_scaling_max_gradient", 100.0);
    ipopt.SetOption("derivative_test", "first-order");

    // 3. solve
    // ipopt.Solve(nlp);



    return 0;
}
