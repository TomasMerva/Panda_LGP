#include <panda_komo_ipopt/komo_lib.h>

KOMO::KOMO(ros::NodeHandle &nh, const int num_joints, const int num_time_slices)
    : _num_joints(num_joints)
     ,_num_time_slices(num_time_slices)
     ,MotionPlanningTools(nh)
{
}

void KOMO::UpdateStates(const std::vector<double> new_start_state, const std::vector<double> new_goal_state)
{
    _q_start = new_start_state;
    _q_goal = new_goal_state;
}


std::vector<std::vector<double>> KOMO::Optimize()
{
    // 1. define a problem
    ifopt::Problem nlp;
    auto g = std::make_shared<AddPointToPointDistanceConstraint>();
    nlp.AddVariableSet(std::make_unique<JointVariables>("x", _num_joints, _num_time_slices, _q_start, _q_goal));
    nlp.AddConstraintSet(g);

    // nlp.AddConstraintSet(std::make_unique<AddPointToPointDistanceConstraint>());
    nlp.AddCostSet(std::make_unique<KOMO_k2_Objective>("k_order=2", _num_joints, _num_time_slices));
    nlp.PrintCurrent();

    g->GetValues();

    // // 2. choose solver and options
    // ifopt::IpoptSolver ipopt;
    // // ipopt.SetOption("linear_solver", "");
    // ipopt.SetOption("jacobian_approximation", "exact");
    // // ipopt.SetOption("jacobian_approximation", "finite-difference-values");
    // ipopt.SetOption("hessian_approximation", "limited-memory");
    // ipopt.SetOption("max_iter", 10000);
    // ipopt.SetOption("tol", 1e-2);
    // ipopt.SetOption("acceptable_tol", 1e-2);
    // // ipopt.SetOption("derivative_test", "first-order");
    
    // // 3. solve
    // ipopt.Solve(nlp);
    // int solver_status = ipopt.GetReturnStatus();    //TODO: handle error result
    // Eigen::VectorXd q = nlp.GetOptVariables()->GetValues();
    
    // // 4. Return solution
    // std::vector<std::vector<double>> q_time_steps;
    // size_t idx=0;
    // for (int col=0; col<_num_time_slices; col++)
    // {
    //     std::vector<double> temp;
    //     for (int row=0; row<_num_joints; row++)
    //     {
    //         temp.push_back(q[idx]);
    //         idx++;
    //     }
    //     q_time_steps.push_back(temp);
    // }
    // std::vector<double> q1, q2, q3, q4, q5, q6, q7;
    // for (int i=0; i<_num_time_slices; i++)
    // {
    //     q1.push_back(q_time_steps[i][0]);
    //     q2.push_back(q_time_steps[i][1]);
    //     q3.push_back(q_time_steps[i][2]);
    //     q4.push_back(q_time_steps[i][3]);
    //     q5.push_back(q_time_steps[i][4]);
    //     q6.push_back(q_time_steps[i][5]);
    //     q7.push_back(q_time_steps[i][6]);
    // }
    // return std::vector<std::vector<double>>{q1, q2, q3, q4, q5, q6, q7};
    return std::vector<std::vector<double>>{{0},{0}};
}
