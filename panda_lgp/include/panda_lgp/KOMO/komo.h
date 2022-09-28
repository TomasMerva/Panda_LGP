#pragma once

#include <cmath>
#include <nlopt.hpp>

#include <panda_lgp/constraints/constraints.h>
#include <panda_lgp/utils/motion_ros_tools.h>
#include <panda_lgp/utils/kinematics.h>
#include <Eigen/Dense>


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
        void AddConstraint(const std::vector<uint> phases_ID, FeatureSymbol g);
        void ClearConstraint(const std::vector<uint> phases_ID);
        void Reset();
        KomoStatus Optimize(LgpLevel level);

        struct Phase
        {
            uint ID;
            std::string symbolic_name;
            uint num_time_slices;
            std::vector<FeatureSymbol> constraints;
            std::vector<double> x_init;
            std::vector<double> x;
        };
        std::vector<Phase> phases;

    private:
        kinematics::Configuration _configuration;

        struct OptimModel
        {
            uint x_dim;
            std::vector<double> x; // first value is init guess
            std::vector<double> lower_bounds;
            std::vector<double> upper_bounds;
        };
        OptimModel _NLP_model;
        
            
};



