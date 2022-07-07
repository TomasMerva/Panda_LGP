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
  const int num_time_slices = 20;


  // DecisionVariables komo("x", num_joints, num_time_slices, q_start, q_goal);
  // Objective objective("k_order=2", num_joints, num_time_slices);
  // // std::cout << komo.GetValues() << std::endl;
  // // objective.Print
  ifopt::Problem nlp;
  nlp.AddVariableSet(std::make_shared<DecisionVariables>("x", num_joints, num_time_slices, q_start, q_goal));
  nlp.AddCostSet(std::make_shared<Objective>("k_order=2", num_joints, num_time_slices));
  nlp.PrintCurrent();


  // 2. choose solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");
  ipopt.SetOption("acceptable_tol", 1e-2);

  // ipopt.SetOption("jacobian_approximation", "finite-difference-values");


  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();

  // Eigen::MatrixXd result(num_joints, num_time_slices);

  std::vector<std::vector<double>> result;
  int idx = 0;
  for (int j=0; j<num_time_slices; j++)
  {
    std::vector<double> temp;
    for (int i=0; i<num_joints; i++)
    {
      temp.push_back(x(idx));
      idx++;
    }
    result.push_back(temp);
  }
  // std::cout << "Results:\n" << result << std::endl;



  // Visualize results
  std::vector<double> time = linspace(0, 5, num_time_slices);
  std::vector<double> q1, q2, q3, q4, q5, q6, q7;
  for (int i=0; i<num_time_slices; i++)
  {
    q1.push_back(result[i][0]);
    q2.push_back(result[i][1]);
    q3.push_back(result[i][2]);
    q4.push_back(result[i][3]);
    q5.push_back(result[i][4]);
    q6.push_back(result[i][5]);
    q7.push_back(result[i][6]);
  }

  
  std::vector<std::vector<double>> data{q1, q2, q3, q4, q5, q6, q7};
  // MultiPlotData(time, data, "Joint_1");
  PlotData(time, q1, "joint1");
  PlotData(time, q2, "joint2");
  PlotData(time, q3, "joint3");
  // PlotData(time, q4, "joint4");
  // PlotData(time, q5, "joint5");
  // PlotData(time, q6, "joint6");
  // PlotData(time, q7, "joint7");



  // for (auto it : row1_vec)
  // {
  //   std::cout << it << " ";
  // }
  // std::cout << std::endl;

  return 0;
}
