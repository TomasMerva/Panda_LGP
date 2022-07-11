#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt_test/test_dec_variable.h>
#include <ifopt_test/test_constraints.h>
#include <ifopt_test/test_objective.h>

using namespace motion_planning;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::vector<double> q_start{0.0, -0.7856, -1.4014, -2.3559, -1.1646, 1.5717, 0.7853};
    std::vector<double> q_goal{-0.2919, 0.1937, -0.1681, -2.086, 0.0369, 2.3322, 0.3197};

    const int num_joints = 7;
    const int num_time_slices = 20;

    // 1. define the problem
    ifopt::Problem nlp;
    nlp.AddVariableSet (std::make_shared<DecisionVariables>("x", num_joints, num_time_slices, q_start, q_goal));
    

    
    return 0;
}