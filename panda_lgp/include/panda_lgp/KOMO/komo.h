#pragma once

#include <cmath>
#include <nlopt.hpp>
#include <chrono>   // for measuring time


#include <panda_lgp/KOMO/komo_objective.h>
#include <panda_lgp/constraints/constraints.h>
#include <panda_lgp/utils/motion_ros_tools.h>
#include <panda_lgp/utils/kinematics.h>
#include <Eigen/Dense>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

enum KomoStatus{
    KS_SolutionFound,
    KS_CannotFindSolution
};

enum LgpLevel{
    SECOND_LEVEL,
    THIRD_LEVEL
};

class KOMO : public MotionROSTools
{
    public:
        KOMO(ros::NodeHandle &nh);
        void SetModel(kinematics::Configuration robot_conf, LgpLevel level);
        void SetTiming(const double num_phases, const double num_time_slices, const double seconds, const double k_order);
        void AddConstraint(const std::vector<uint> phases_ID, Constraint::ConstraintSymbol g);
        void ClearConstraint(const std::vector<uint> phases_ID);
        void Reset();
        KomoStatus Optimize(LgpLevel level);

        void AddJointLimits(std::vector<double> &lower_bounds, std::vector<double> &upper_bounds);

        friend std::ostream& operator<< (std::ostream& out, const KOMO& obj); 


        struct Phase
        {
            uint ID;
            std::string symbolic_name;
            uint num_time_slices; // maybe delete
            std::vector<nlopt::mfunc> constraints;
            std::vector<Constraint::ConstraintData> constraints_data;
            std::vector<double> x;
            std::vector<double> x_init;
            std::vector<std::vector<double>> q_trajectory;  // trajectory from this phase to the next phase
            std::vector<double> lower_bounds;
            std::vector<double> upper_bounds;
        };
        std::vector<Phase> phases;


    private:
        KomoStatus SecondLevel();
        KomoStatus ThirdLevel();
        void VerboseSolver(const nlopt::result &result); 
        std::vector<double> SetInitialGuess(const std::vector<double> &start_state, const std::vector<double> &goal_state);

        uint _x_dim;
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

