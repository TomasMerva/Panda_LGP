#pragma once
#include "ceres/ceres.h"

class Collision_FeatureCost : public ceres::CostFunction {
    public:
        Collision_FeatureCost(std::vector<double> obstacle_position);
        virtual bool Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const;
    private:
        std::vector<double> _obstacle_pos;        
};

class Collision_Langrange : public ceres::CostFunction
{
    public:
        Collision_Langrange(const int idx, std::vector<double> obstacle_position);
        virtual bool Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const;
        
        static std::vector<double> lambda; 
    private:
        std::vector<double> _obstacle_pos;   
        int _idx;
};