#pragma once

#include "ceres/ceres.h"
#include "glog/logging.h"
#include <iomanip>
#include <ceres_testy/komo_objective.h>
#include <ceres_testy/collision_constraint.h>
#include <ceres_testy/mu_loss_function.h>

#include <chrono>

class Solver
{
    public:
        Solver(const int num_timestep_var, const int num_timesteps);
        void Solve();
        void CreateInitGuess(bool linspace_flag);
        void SetBoundaries();

        // Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> x;
        Eigen::ArrayXXd x;
        std::vector<double> q_start;
        std::vector<double> q_goal;
        
    private:
        int _num_timesteps;
        int _num_timestep_var;
        ceres::Problem* _ceresProblem;

        // Inequality constraints parameters
        std::vector<double> _lambda;
};


// -----------------------------------------
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