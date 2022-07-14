#include <panda_motion_planning/komo_lib.h>

///////////////////////////////////////////////////////////////////////
/// \brief KOMO::KOMO Setting sizes of variables
/// \param
///////////////////////////////////////////////////////////////////////
KOMO::KOMO(const int num_joints, const int num_time_slices, const int k_order)
    : _num_joints(num_joints)
    , _num_time_slices(num_time_slices)
{    
}


///////////////////////////////////////////////////////////////////////
/// \brief KOMO::UpdateStates Update the start and goal state
/// \param
///////////////////////////////////////////////////////////////////////
void KOMO::UpdateStates(const std::vector<double> new_start_state, const std::vector<double> new_goal_state)
{
    _start_state = new_start_state;
    _goal_state = new_goal_state;
}

///////////////////////////////////////////////////////////////////////
/// \brief KOMO::Optimize() Construct NLP and use AUGLAG solver to find an optimum
/// \param 
///////////////////////////////////////////////////////////////////////
std::vector<std::vector<double>> KOMO::Optimize()
{
    std::unique_ptr<JointsVariable> x = std::make_unique<JointsVariable>(_num_joints, _num_time_slices);
    std::unique_ptr<KOMO_k2> objective = std::make_unique<KOMO_k2>(_num_joints, _num_time_slices); 

    // 1. choose solvers
    nlopt::opt opt(nlopt::algorithm::AUGLAG, x->GetNumVariables());
    nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND, x->GetNumVariables());
    local_opt.set_xtol_abs(_local_x_tol);
    opt.set_xtol_abs(_x_tol);
    opt.set_local_optimizer(local_opt);

    // 2. set bounds
    x->SetBoundaryConstraints(_start_state, _goal_state);
    opt.set_lower_bounds(x->GetLowerBounds());
    opt.set_upper_bounds(x->GetUpperBounds());

    // 3. set an objective function
    opt.set_min_objective(objective->GetCost, NULL);


    // 4. set constraints
    // TODO: create a method for adding constraints

    // 5. set an initial guess
    std::vector<double> q = x->InitialGuess(_start_state, _goal_state);

    // 6. optimize
    double min_obj_value;
    auto start_time = high_resolution_clock::now();
    try
    {
        nlopt::result result = opt.optimize(q, min_obj_value);
        auto finish_time = high_resolution_clock::now();
        duration<double, std::milli> ms_double = finish_time - start_time;
        std::cout << "------------------------------------------\n";
        std::cout << "Number of iterations: \t=\t" << objective->num_iterations << "\n";
        std::cout << "Total ms in NLOPT: \t=\t" << ms_double.count() << " ms\n";
        std::cout << "EXIT: Optimal Solution Found.\n--------" << std::endl;  
    }
    catch(std::exception &e) 
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    // --------------
        // Return results
        std::vector<std::vector<double>> q_time_steps;
        size_t idx=0;
        for (int col=0; col<_num_time_slices; col++)
        {
            std::vector<double> temp;
            for (int row=0; row<_num_joints; row++)
            {
                temp.push_back(q[idx]);
                idx++;
            }
            q_time_steps.push_back(temp);
        }
        std::vector<double> q1, q2, q3, q4, q5, q6, q7;
        for (int i=0; i<_num_time_slices; i++)
        {
            q1.push_back(q_time_steps[i][0]);
            q2.push_back(q_time_steps[i][1]);
            q3.push_back(q_time_steps[i][2]);
            q4.push_back(q_time_steps[i][3]);
            q5.push_back(q_time_steps[i][4]);
            q6.push_back(q_time_steps[i][5]);
            q7.push_back(q_time_steps[i][6]);
        }
        return std::vector<std::vector<double>>{q1, q2, q3, q4, q5, q6, q7};
}

///////////////////////////////////////////////////////////////////////
/// \brief KOMO::GetJointLimits Return the set joint limits
/// \param 
///////////////////////////////////////////////////////////////////////
std::vector<std::pair<double, double>> KOMO::GetJointLimits()
{
    std::pair<double,double> q1_limit = std::make_pair(-2.8973, 2.8973);
    std::pair<double,double> q2_limit = std::make_pair(-1.7628, 1.7628);
    std::pair<double,double> q3_limit = std::make_pair(-2.8973, 2.8973);
    std::pair<double,double> q4_limit = std::make_pair(-3.0718, -0.0698);
    std::pair<double,double> q5_limit = std::make_pair(-2.8973, 2.8973);
    std::pair<double,double> q6_limit = std::make_pair(-0.0175, 3.7525);
    std::pair<double,double> q7_limit = std::make_pair(-2.8973, 2.8973);

    std::vector<std::pair<double,double>> joint_limits{q1_limit, q2_limit, q3_limit, q4_limit, q5_limit, q6_limit, q7_limit};
    return joint_limits;
}
