#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <ifopt/constraint_set.h>

#include <panda_komo_ipopt/utils/panda_kinematics.h>


using Vector2d = Eigen::Vector2d;


class AddPointToPointDistanceConstraint : public ifopt::ConstraintSet {
    public:
        AddPointToPointDistanceConstraint();
        // This constraint set just contains 1 constraint, however generally
        // each set can contain multiple related constraints.
        AddPointToPointDistanceConstraint(const std::string& name, const int num_time_slices, const double tolerance);

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

        static autodiff::MatrixXreal DH_matrix(const autodiff::real a, const autodiff::real d, 
                                         const autodiff::real alpha, const autodiff::real theta);
        static autodiff::real ComputePointToPointDistanceConstraint
            (autodiff::VectorXreal q, autodiff::VectorXreal obj_position);                          


    private:
        double _tolerance;
};