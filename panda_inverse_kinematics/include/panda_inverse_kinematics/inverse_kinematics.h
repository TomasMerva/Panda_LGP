#pragma once
#include <vector>

#include <ros/ros.h>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <panda_inverse_kinematics/variables/joints.h>
#include <panda_inverse_kinematics/costs/quadratic_error_cost.h>
#include <panda_inverse_kinematics/constraints/position_constraint.h>
#include <panda_inverse_kinematics/constraints/orientation_constraint.h>


enum FeatureSymbol{
    FS_none = 0,
    FS_PositionConstraint = 1,
    FS_PointToPointDistance = 2,
    FS_OrientationConstraint = 3,
};


class InverseKinematics
{
    public:
        InverseKinematics();
        std::vector<double> Solve();
        void AddConstraint(FeatureSymbol feature);
        void SetInitialGuess(std::vector<double> q0);
        void AddPositionConstraint(std::vector<double> p_AQ_bounds);
        void AddOrientationConstraint(std::vector<double> rpy);

    private:
        const int _num_joints;
        std::vector<double> _q_start;
        std::vector<FeatureSymbol> _g_list;
        std::vector<double> _p_AQ_bounds;
        std::vector<double> _rpy;

};