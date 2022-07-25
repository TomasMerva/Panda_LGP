#pragma once
#include <math.h>
#include <vector>
#include <chrono>   // for measuring time
#include <memory>   // unique_ptr

#include <ros/ros.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <panda_komo_ipopt/variables/joints.h>
#include <panda_komo_ipopt/objectives/komo_k2.h>
#include <panda_komo_ipopt/constraints/add_point_to_point_distance.h>

#include <panda_komo_ipopt/utils/motion_planning_tools.h>

class KOMO : public MotionPlanningTools
{
    public:
        KOMO(ros::NodeHandle &nh, const int num_joints, const int num_time_slices);

        std::vector<std::vector<double>> Optimize();
        void UpdateStates(const std::vector<double> new_start_state, const std::vector<double> new_goal_state);


    private:
        const int _num_joints;
        const int _num_time_slices;

        std::vector<double> _q_start;
        std::vector<double> _q_goal;
};
