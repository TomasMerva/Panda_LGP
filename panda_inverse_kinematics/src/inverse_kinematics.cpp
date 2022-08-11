#include <panda_inverse_kinematics/inverse_kinematics.h>


InverseKinematics::InverseKinematics()
    : _num_joints(7)
{
    _q_start = std::vector<double>(7);
}

void InverseKinematics::AddConstraint(FeatureSymbol feature)
{
    _g_list.push_back(feature);
}

void InverseKinematics::SetInitialGuess(std::vector<double> q0)
{
    _q_start = q0;
}

void InverseKinematics::AddPositionConstraint(std::vector<double> p_AQ_bounds)
{
    _p_AQ_bounds = p_AQ_bounds;
}

void InverseKinematics::AddOrientationConstraint(std::vector<double> rpy)
{
    _rpy = rpy;
}


std::vector<double> InverseKinematics::Solve()
{
    // 1. define a problem
    ifopt::Problem nlp;
    nlp.AddVariableSet(std::make_unique<JointVariables>("x", _num_joints, _q_start));
    nlp.AddCostSet(std::make_unique<QuadraticErrorCost>("QuadraticErrorCost", _num_joints, _q_start));
    nlp.AddConstraintSet(std::make_unique<PositionConstraint>("PositionConstraint", _p_AQ_bounds, _p_AQ_bounds));
    nlp.AddConstraintSet(std::make_unique<OrientationConstraint>("OrientationConstraint", _rpy, 0.01));

    nlp.PrintCurrent();

    // 2. choose solver and options
    ifopt::IpoptSolver ipopt;
    // ipopt.SetOption("linear_solver", "");
    // ipopt.SetOption("jacobian_approximation", "exact");
    ipopt.SetOption("jacobian_approximation", "finite-difference-values");
    ipopt.SetOption("hessian_approximation", "limited-memory");
    ipopt.SetOption("max_iter", 10000);
    ipopt.SetOption("tol", 1e-2);
    ipopt.SetOption("acceptable_tol", 1e-2);
    ipopt.SetOption("constr_viol_tol", 1e-2);

    // ipopt.SetOption("nlp_scaling_max_gradient", 100.0);
    // ipopt.SetOption("derivative_test", "first-order");


    // 3. solve
    ipopt.Solve(nlp);
    Eigen::VectorXd q = nlp.GetOptVariables()->GetValues();

    // 4. Return solution
    std::vector<double> result(q.data(), q.data()+q.rows());
    return result;
}