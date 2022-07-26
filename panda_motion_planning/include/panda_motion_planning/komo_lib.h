#pragma once

#include <math.h>
#include <vector>
#include <chrono>   // for measuring time
#include <memory>   // unique_ptr

#include <nlopt.hpp>
#include <panda_motion_planning/variables/joints.h>
#include <panda_motion_planning/objective/komo_k2.h>
#include <panda_motion_planning/utils/motion_planning_tools.h>
#include <panda_motion_planning/constraints/add_point_to_point_distance.h>


using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

enum FeatureSymbol{
    FS_none = -1,
    FS_PointToPointDistance
};


typedef struct
{
    double obj_x = 0.6;
    double obj_y = 0.0;
    double obj_z = 0.1;
    double dist_tol = 0.05;
} constraint_data;



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

        // TODO: add time step range
        void AddConstraint(FeatureSymbol);
        void ClearConstraint();


    private:
        const int _num_joints;
        const int _num_time_slices;
        const int _k_order = 2;

        const double _local_x_tol = 1e-6;
        const double _x_tol = 1e-6;
        bool nlopt_success = false;

        std::vector<double> _start_state;
        std::vector<double> _goal_state;
        std::vector<std::pair<double, double>> _joint_limits;

        std::vector<FeatureSymbol> _constraint_list;

        std::vector<double> _init_start_guess;
};

