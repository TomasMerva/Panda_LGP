#pragma once

#include <cmath>
#include <nlopt.hpp>

#include <panda_lgp/utils/motion_ros_tools.h>
#include <panda_lgp/utils/kinematics.h>

enum FeatureSymbol{
    FS_none,
    FS_PointToPointDistance,
    FS_FixedOrientationAxis
};


enum KomoStatus{
    KS_SolutionFound,
    KS_CannotFindSolution
};


class KOMO : public MotionROSTools
{
    public:
        KOMO(ros::NodeHandle &nh);
        void SetModel(kinematics::Configuration robot_conf);
        void SetTiming(const double num_phases, const double num_time_slices, const double seconds, const double k_order);
        void AddConstraint(const std::vector<uint> phases_ID, FeatureSymbol g);
        void ClearConstraint(const std::vector<uint> phases_ID);
        void Reset();
        KomoStatus Optimize();

        struct Phase
        {
            uint ID;
            double num_time_slices;
            std::vector<FeatureSymbol> constraints;
            std::vector<double> x_init;
            std::vector<double> x;
        };
        std::vector<Phase> phases;

    private:
        kinematics::Configuration _configuration;
            
};



