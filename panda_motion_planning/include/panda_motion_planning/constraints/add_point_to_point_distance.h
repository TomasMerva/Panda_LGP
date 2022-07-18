#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <panda_motion_planning/panda_kinematics.h>


class AddPointToPointDistanceConstraint : public panda_kinematics::Kinematics
{
    public:
        AddPointToPointDistanceConstraint(/* args */);
        ~AddPointToPointDistanceConstraint();

        double GetValues(const std::vector<double> &x, std::vector<double> &grad, void *data);
        // double GetValues(const std::vector<double> &x);
        void FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac);

    private:
        /* data */
};

