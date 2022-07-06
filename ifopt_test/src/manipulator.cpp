#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt_test/test_dec_variable.h>
#include <ifopt_test/test_constraints.h>
#include <ifopt_test/test_objective.h>

#include "gnuplot-iostream.h"

using namespace motion_planning;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulator");
  ros::NodeHandle nh;

  std::vector<double> q_start{0.0, -0.7856, -1.4014, -2.3559, -1.1646, 1.5717, 0.7853};
  std::vector<double> q_goal{-0.2919, 0.1937, -0.1681, -2.086, 0.0369, 2.3322, 0.3197};
  const int num_joints = 7;
  const int num_time_slices = 5;


  // DecisionVariables komo("x", num_joints, num_time_slices, q_start, q_goal);
  // komo.GetValues();
  ifopt::Problem nlp;
  nlp.AddVariableSet(std::make_shared<DecisionVariables>("x", num_joints, num_time_slices, q_start, q_goal));
  nlp.AddCostSet(std::make_shared<Objective>("k_order=2", num_joints, num_time_slices));
  nlp.PrintCurrent();


  // 2. choose solver and options
  // ifopt::IpoptSolver ipopt;
  // ipopt.SetOption("linear_solver", "mumps");
  // ipopt.SetOption("jacobian_approximation", "true");
  // ipopt.SetOption("acceptable_tol", 1e-3);

  // // ipopt.SetOption("jacobian_approximation", "finite-difference-values");


  // 3 . solve
  // ipopt.Solve(nlp);
  // Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  // Eigen::MatrixXd result(num_joints, num_time_slices);
  // int idx = 0;
  // for (int j=0; j<num_time_slices; j++)
  // {
  //   for (int i=0; i<num_joints; i++)
  //   {
  //     result(i,j) = x(idx);
  //     idx++;
  //   }
  // }
  // std::cout << "Results:\n" << result << std::endl;



  // Visualize results
  // std::vector<double> time = linspace(0, 5, num_time_slices);
  // std::vector<double> y(num_time_slices, 5);
  // std::vector<double> row1_vec(result.row(0).data(), result.row(0).data() + 20);
  // PlotData(time, row1_vec, "Joint_1");

  return 0;
}
