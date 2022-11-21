#pragma once
#include "ceres/ceres.h"


class KOMO_FeatureCost : public ceres::CostFunction {
    public:
        KOMO_FeatureCost();
        virtual bool Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const;
                                
};
