#pragma once

#include <math.h>
#include <vector>
#include <chrono>   // for measuring time
#include <memory>   // unique_ptr

#include <nlopt.hpp>
#include <panda_motion_planning/variables/joints.h>
#include <panda_motion_planning/objective/komo_k2.h>
#include <panda_motion_planning/motion_planning_tools.h>


using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


// TODO: k-order is not implemented, it is always k_order = 2
// TODO: flag for not finding trajectory
class KOMO : public MotionPlanningTools
{
    public:
        KOMO(ros::NodeHandle &nh, const int num_joints, const int num_time_slices, const int k_order);
        ~KOMO(){};
        void UpdateStates(const std::vector<double> new_start_state, const std::vector<double> new_goal_state);
        std::vector<std::vector<double>> Optimize();
        void GetReport();   // TODO: 
        std::vector<std::pair<double, double>> GetJointLimits();    //TODO



    private:
        const int _num_joints;
        const int _num_time_slices;
        const int _k_order = 2;

        const double _local_x_tol = 0.01;
        const double _x_tol = 1e-4;

        std::vector<double> _start_state;
        std::vector<double> _goal_state;
        std::vector<std::pair<double, double>> _joint_limits;
};

