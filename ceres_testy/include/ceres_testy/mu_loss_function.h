#pragma once

#include "ceres/ceres.h"



class MuLoss : public ceres::LossFunction {
    public:
        MuLoss(const LossFunction* rho, double a, ceres::Ownership ownership);
        MuLoss(const MuLoss&) = delete;
        void operator=(const MuLoss&) = delete;
        ~MuLoss() override {
            if (ownership_ == ceres::DO_NOT_TAKE_OWNERSHIP) {
            rho_.release();
            }
        }

        void Evaluate(double s, double rho[3]) const;
        
        static double nu; 

    private:
        std::unique_ptr<const ceres::LossFunction> rho_;
        const ceres::Ownership ownership_;  
        const double a_;
};