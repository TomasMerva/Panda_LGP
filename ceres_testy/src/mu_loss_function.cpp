#include "ceres_testy/mu_loss_function.h"

double MuLoss::nu = 1;

MuLoss::MuLoss(const LossFunction* rho, double a, ceres::Ownership ownership)
    : rho_(rho), a_(a), ownership_(ownership)
{
    MuLoss::nu = a;
}

void 
MuLoss::Evaluate(double s, double rho[3]) const
{
    if (rho_.get() == nullptr) {
        rho[0] = MuLoss::nu * s;
        rho[1] = MuLoss::nu;
        rho[2] = 0.0;
    }
}