#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <ifopt/constraint_set.h>
#include <panda_inverse_kinematics/utils/panda_kinematics.h>

using Vector2d = Eigen::Vector2d;


class OrientationConstraint : public ifopt::ConstraintSet {
    public:
        OrientationConstraint(const std::string& name, const std::vector<double> rpy, const double theta_bound);

        // // The constraint value minus the constant value "1", moved to bounds.
        VectorXd GetValues() const override;

        // The only constraint in this set is an equality constraint to 1.
        // Constant values should always be put into GetBounds(), not GetValues().
        // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
        VecBound GetBounds() const override;

        // // This function provides the first derivative of the constraints.
        // // In case this is too difficult to write, you can also tell the solvers to
        // // approximate the derivatives by finite differences and not overwrite this
        // // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
        // // Attention: see the parent class function for important information on sparsity pattern.
        void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
               
        static std::vector<double> g_jac;
        static double _theta_bound;
        static std::vector<double> _rpy;

    private:

};