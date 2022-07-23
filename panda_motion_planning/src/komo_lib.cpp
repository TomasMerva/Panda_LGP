#include <panda_motion_planning/komo_lib.h>

///////////////////////////////////////////////////////////////////////
/// \brief KOMO::KOMO Setting sizes of variables
/// \param
///////////////////////////////////////////////////////////////////////
KOMO::KOMO(ros::NodeHandle &nh, const int num_joints, const int num_time_slices, const int k_order)
    : _num_joints(num_joints)
    , _num_time_slices(num_time_slices)
    , MotionPlanningTools(nh)
{    
    _init_start_guess = {0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397};
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
    // nlopt::algorithm::LD_AUGLAG_EQ
    nlopt::opt opt(nlopt::algorithm::LD_AUGLAG, x->GetNumVariables());
    nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND, x->GetNumVariables());
    // nlopt::opt local_opt(nlopt::algorithm::LD_MMA, x->GetNumVariables());
    //LD_SLSQP
    local_opt.set_xtol_rel(_local_x_tol);
    opt.set_local_optimizer(local_opt);
    opt.set_xtol_rel(_x_tol);
    opt.set_maxtime(60);
    
    // 2. set bounds
    x->SetBoundaryConstraints(_start_state, _goal_state);
    opt.set_lower_bounds(x->GetLowerBounds());
    opt.set_upper_bounds(x->GetUpperBounds());

    // 3. set an objective function
    opt.set_min_objective(objective->GetCost, NULL);


    // 4. set constraints
    AddPointToPointDistanceData constraint_data[_num_time_slices];
    for (int timestep=1; timestep < (_num_time_slices-1); timestep++)
    {
        constraint_data[timestep-1] = {timestep, 0.5, 0.0, 0.125, 0.6};
        opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[timestep-1], 1e-8);
    }

    // opt.set_ftol_abs(1e-6);
    // opt.set_ftol_rel(1e-6);
    // std::cout << opt.get_ftol_abs() << " " << opt.get_ftol_rel() << std::endl; 
    
    // for (auto g : _constraint_list)
    // {
    //     switch (g)
    //     {
    //         case FS_none:
    //             break;
            
    //         case FS_PointToPointDistance:
    //             for (int timestep=1; timestep < (_num_time_slices-1); timestep++)
    //             {
    //                 constraint_data[timestep-1] = {timestep, 0.5, 0.0, 0.125, 0.5};
    //                 opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[timestep-1], 1e-8);
    //             }
    //             break;

    //         default:
    //             break;
    //     }
    // }

    

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
        std::cout << "NLOPT status: " << result << std::endl;
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

///////////////////////////////////////////////////////////////////////
/// \brief KOMO::AddObjective Add kinematic constraint
/// \param 
///////////////////////////////////////////////////////////////////////
void KOMO::AddConstraint(FeatureSymbol FS)
{
    _constraint_list.push_back(FS);
}

///////////////////////////////////////////////////////////////////////
/// \brief KOMO::ClearObjectives Delete all constraints
/// \param 
///////////////////////////////////////////////////////////////////////
void KOMO::ClearConstraint()
{
    _constraint_list.clear();   
}