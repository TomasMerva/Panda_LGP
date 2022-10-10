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


        struct Phase
        {
            uint ID;
            std::string symbolic_name;
            uint num_time_slices;
            // std::vector<ConstraintSymbol> constraints;
            // std::vector<std::function<double(const std::vector<double> &x, std::vector<double> &grad, void *data)>> constraints;
            std::vector<nlopt::vfunc> constraints;
            std::vector<Constraint::ConstraintData> constraints_data;
            std::vector<double> x_init;
            std::vector<double> x;
            std::vector<double> lower_bounds;
            std::vector<double> upper_bounds;
        };
        std::vector<Phase> phases;


    private:
        KomoStatus SecondLevel();
        KomoStatus ThirdLevel();
        void VerboseSolver(const nlopt::result &result); 

        uint _x_dim;
};



