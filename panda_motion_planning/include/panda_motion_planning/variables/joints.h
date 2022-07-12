#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>


class JointsVariable
{
public:
    JointsVariable(const int num_variables, const int num_time_slices);
    ~JointsVariable();

    std::vector<double> GetVariables();
    std::vector<double> GetLowerBounds();
    std::vector<double> GetUpperBounds();
    std::vector<double> InitialGuess(const std::vector<double> start_state, const std::vector<double> goal_state);

    void SetJointLimits();
    void SetBoundaryConstraints(const std::vector<double> start_state, const std::vector<double> goal_state);

    int GetNumVariables();


private:
    Eigen::MatrixXd _q;
    std::vector<double> _lower_bounds;
    std::vector<double> _upper_bounds;
    const int _num_vector_variables;
    const int _num_time_slices;
    const int _num_variables;
};




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