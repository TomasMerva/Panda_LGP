#include <panda_lgp/KOMO/komo.h>

///////////////////////////////////////////////////////////////////////
/// @brief Initialize publishers for RViz and franka controllers
/// @param nh ROS NodeHandle for creating publishers
///////////////////////////////////////////////////////////////////////
KOMO::KOMO(ros::NodeHandle &nh) 
    : MotionROSTools(nh)
{
}

///////////////////////////////////////////////////////////////////////
/// @brief Frankly, have no idea if this is needed
/// @param robot_conf Structure representing the robot conf, joint limits, ...
///////////////////////////////////////////////////////////////////////
void 
KOMO::SetModel(kinematics::Configuration robot_conf, LgpLevel level)
{    
 
}

///////////////////////////////////////////////////////////////////////
/// @brief Creates phases based on the parameters
/// @param num_phases Number of phases
/// @param num_time_slices Number of time slices between phases
/// @param seconds Time between each trajectory
/// @param k_order KOMO parameter for the objective function
///////////////////////////////////////////////////////////////////////
void 
KOMO::SetTiming(const double num_phases, const double num_time_slices, 
                     const double seconds, const double k_order)
{
    for (uint i = 0; i<num_phases; ++i)
    {
        // phases.push_back(Phase{i, "pick" , num_time_slices, std::vector<ConstraintSymbol>{ConstraintSymbol::FS_none}});
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Add constraints to the phases based on given ID
/// @param phases_ID Vector of phases' ID
/// @param ConstraintSymbol Enum for different constraints
///////////////////////////////////////////////////////////////////////
void 
KOMO::AddConstraint(const std::vector<uint> phases_ID, Constraint::ConstraintSymbol g)
{
    for (auto id : phases_ID)
    {
        // if (phases[id].constraints[0] == ConstraintSymbol::FS_none ) phases[id].constraints.clear();
        // phases[id].constraints.push_back(g);
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Remove all constraints from the phases based on given ID
/// @param phases_ID Vector of phases' ID
///////////////////////////////////////////////////////////////////////
void 
KOMO::ClearConstraint(const std::vector<uint> phases_ID)
{
    for (auto id : phases_ID)
    {
        phases[id].constraints.clear();
        // phases[id].constraints.push_back(ConstraintSymbol::FS_none);
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Delete all phases
/// @param
///////////////////////////////////////////////////////////////////////
void 
KOMO::Reset()
{
    phases.clear();
}


///////////////////////////////////////////////////////////////////////
/// @brief Find a solution
/// @param
///////////////////////////////////////////////////////////////////////
KomoStatus 
KOMO::Optimize(LgpLevel level)
{
    if (level == LgpLevel::SECOND_LEVEL)
    {
        return SecondLevel();
    }
    else if(level == LgpLevel::THIRD_LEVEL)
    {
        return ThirdLevel();
    }
    else
    {
        ROS_ERROR("No level defined!");
        return KomoStatus::KS_CannotFindSolution;
    }
}


///////////////////////////////////////////////////////////////////////
/// @brief Set boundaries for joint angles
/// @param
///////////////////////////////////////////////////////////////////////
void 
KOMO::AddJointLimits(std::vector<double> &lower_bounds, std::vector<double> &upper_bounds)
{
    // q1
    lower_bounds[0] = -2.8973;
    upper_bounds[0] = 2.8973;
    // q2
    lower_bounds[1] = -1.7628;
    upper_bounds[1] = 1.7628;
    // q3
    lower_bounds[2] = -2.8973;
    upper_bounds[2] = 2.8973;
    // q4
    lower_bounds[3] = -3.0718;
    upper_bounds[3] = -0.0698;
    // q5
    lower_bounds[4] = -2.8973;
    upper_bounds[4] = 2.8973;
    // q6
    lower_bounds[5] = -0.0175;
    upper_bounds[5] = 3.7525;
    // q7
    lower_bounds[6] = -2.8973;
    upper_bounds[6] = 2.8973;
}

///////////////////////////////////////////////////////////////////////
/// @brief Second level
/// @param
///////////////////////////////////////////////////////////////////////
KomoStatus 
KOMO::SecondLevel()
{
    uint x_phase_dim = 7 + 6; //q(7) + 6DoF (manipulation frame)

    // ---- 1. Choose solver and set decision variables LD_TNEWTON_PRECOND_RESTART ----
    nlopt::opt opt(nlopt::algorithm::AUGLAG, (phases.size()-1)*x_phase_dim);
    nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, (phases.size()-1)*x_phase_dim);
    opt.set_xtol_rel(1e-6);
    opt.set_xtol_abs(1e-6);  
    opt.set_local_optimizer(local_opt);

    // ---- 2. Set boundaries ----
    // - convert boundaries from all phases into one vector
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    for (uint idx=1; idx<phases.size(); ++idx)
    {
        lower_bounds.insert(lower_bounds.end(), phases[idx].lower_bounds.begin(), phases[idx].lower_bounds.end());
        upper_bounds.insert(upper_bounds.end(), phases[idx].upper_bounds.begin(), phases[idx].upper_bounds.end());
    }
    opt.set_lower_bounds(lower_bounds);
    opt.set_upper_bounds(upper_bounds);

    // ---- 3. Set objective function ----
    KOMO_k2::ObjectiveData objective;
    objective.num_phase_variables = x_phase_dim;
    objective.num_phases = phases.size()-1;
    objective.num_iterations = 0;
    opt.set_min_objective(KOMO_k2::GetCost, &objective);

    // ---- 4. Set constraints ----
    // Set Manipulation frame constraint
    const std::vector<double> k_tolerance_manip{1e-6, 1e-6, 1e-6};
    Constraint::ConstraintData g_manip_frame_data;
    g_manip_frame_data.num_phase_variables = x_phase_dim;
    g_manip_frame_data.idx = 0;
    opt.add_equality_mconstraint(Constraint::ManipulationFrame, &g_manip_frame_data, k_tolerance_manip);
    


    const double k_tolerance = 1e-8;
    for (auto &phase : phases)
    {
        for (uint i=0; i<phase.constraints.size(); ++i)
        {
            std::cout << phase.constraints[i] << std::endl;
            opt.add_inequality_constraint(phase.constraints[i], &phase.constraints_data[i], k_tolerance);
        }
    }
    
    // ---- 5. Set an initial guess
    std::vector<double> x;
    for (uint idx=1; idx<phases.size(); ++idx)
    {
        x.insert(x.end(), phases[idx].x_init.begin(), phases[idx].x_init.end());
    }

    // ---- 6. Optimize
    double min_obj_value;
    auto start_time = high_resolution_clock::now();
    nlopt::result result;
    try
    {
        result = opt.optimize(x, min_obj_value);
        auto finish_time = high_resolution_clock::now();
        std::cout << "------------------------------------------\n";
        duration<double, std::milli> ms_double = finish_time - start_time;
        std::cout << "Number of iterations: \t=\t" << objective.num_iterations << "\n";
        std::cout << "Total ms in NLOPT: \t=\t" << ms_double.count() << " ms\n";
        VerboseSolver(result);
        // Convert to phases
        uint begin = 0;
        uint end = objective.num_phase_variables;
        for (uint idx=1; idx<phases.size(); ++idx)
        {
            phases[idx].x.insert(phases[idx].x.begin(), x.begin()+begin, x.begin()+end);
            begin = end;
            end += end;
        }
        return KomoStatus::KS_SolutionFound;
    }
    catch(std::exception &e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
        std::cout << "result code: " << result << "\n";
        std::cout << "Number of iterations: \t=\t" << objective.num_iterations << "\n";
        return KomoStatus::KS_CannotFindSolution;
    }
}


///////////////////////////////////////////////////////////////////////
/// @brief Third level
/// @param
///////////////////////////////////////////////////////////////////////
KomoStatus 
KOMO::ThirdLevel()
{
    uint x_phase_dim = 7; //q(7)
    uint phase_time_steps = 20;    

    for (uint idx=0; idx<phases.size()-1; ++idx)
    {
        std::cout << "Trajectory between: " << phases[idx].symbolic_name << " and " << phases[idx+1].symbolic_name << "\n";

        // ---- 1. Choose solver and set decision variables LD_TNEWTON_PRECOND_RESTART ----
        nlopt::opt opt(nlopt::algorithm::AUGLAG, x_phase_dim*phase_time_steps);
        nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, x_phase_dim*phase_time_steps);
        opt.set_xtol_rel(1e-6);
        opt.set_xtol_abs(1e-6);  
        opt.set_local_optimizer(local_opt);

        // ---- 2. Set boundaries ----
        std::vector<double> lower_bounds;
        std::vector<double> upper_bounds;
        for (uint t=0; t<phase_time_steps; ++t)
        {
            // Start state in a phase defined by 2.level or an user
            if (t==0)
            {
                lower_bounds.insert(lower_bounds.end(), phases[idx].x.begin(), phases[idx].x.begin()+x_phase_dim);
                upper_bounds.insert(upper_bounds.end(), phases[idx].x.begin(), phases[idx].x.begin()+x_phase_dim);
            }
            // End state in a phase defined by 2.level or an user
            // I am taking from phase[t+1]
            else if (t==phase_time_steps-1)
            {
                lower_bounds.insert(lower_bounds.end(), phases[idx+1].x.begin(), phases[idx+1].x.begin()+x_phase_dim);
                upper_bounds.insert(upper_bounds.end(), phases[idx+1].x.begin(), phases[idx+1].x.begin()+x_phase_dim);
            }
            else
            {
                lower_bounds.insert(lower_bounds.end(), phases[idx].lower_bounds.begin(), phases[idx].lower_bounds.begin()+x_phase_dim);
                upper_bounds.insert(upper_bounds.end(), phases[idx].upper_bounds.begin(), phases[idx].upper_bounds.begin()+x_phase_dim);
            }
        }
        opt.set_lower_bounds(lower_bounds);
        opt.set_upper_bounds(upper_bounds);

        // ---- 3. Set objective function ----
        KOMO_k2::ObjectiveData objective;
        objective.num_phase_variables = x_phase_dim;
        objective.num_phases = 20; // 20 timesteps between two points
        objective.num_iterations = 0;
        opt.set_min_objective(KOMO_k2::GetCost, &objective);

        // ---- 4. Set constraints ----

        // ---- 5. Set an initial guess ----
        std::vector<double> x = SetInitialGuess(phases[idx].x, phases[idx+1].x);
        
        // ---- 6. Optimize ----
        double min_obj_value;
        auto start_time = high_resolution_clock::now();
        nlopt::result result;
        try
        {
            result = opt.optimize(x, min_obj_value);
            auto finish_time = high_resolution_clock::now();
            std::cout << "------------------------------------------\n";
            duration<double, std::milli> ms_double = finish_time - start_time;
            std::cout << "Number of iterations: \t=\t" << objective.num_iterations << "\n";
            std::cout << "Total ms in NLOPT: \t=\t" << ms_double.count() << " ms\n";
            VerboseSolver(result);
            std::cout << "x size: " << x.size() << std::endl;
            
            // ---- 7. Save results ----
            uint begin = 0;
            uint end = x_phase_dim;
            for (uint t=0; t<phase_time_steps; ++t)
            {
                std::vector<double> temp(x.begin()+begin, x.begin()+end);
                phases[idx].q_trajectory.push_back(temp);
                begin = end;
                end += x_phase_dim;
            }
        }
        catch(std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
            std::cout << "result code: " << result << "\n";
            std::cout << "Number of iterations: \t=\t" << objective.num_iterations << "\n";
            return KomoStatus::KS_CannotFindSolution;
        }
    }
    return KomoStatus::KS_SolutionFound;
}

///////////////////////////////////////////////////////////////////////
/// @brief Translates nlopt code into message
/// @param result Return value from optimize()
///////////////////////////////////////////////////////////////////////
void 
KOMO::VerboseSolver(const nlopt::result &result)
{
    switch (result)
    {
        // Positive messages
        case 1:
            std::cout << "NLOPT status: Generic success return value.\n";
            std::cout << "EXIT: Optimal Solution Found.\n--------\n\n";
            break;
        case 2:
            std::cout << "NLOPT status: Optimization stopped because `stopval` was reached.\n";
            std::cout << "EXIT: Optimal Solution Found.\n--------\n\n";
            break;
        case 3:
            std::cout << "NLOPT status: Optimization stopped because `ftol_rel` or `ftol_abs` was reached.\n";
            std::cout << "EXIT: Optimal Solution Found.\n--------\n\n";
            break;
        case 4:
            std::cout << "NLOPT status: Optimization stopped because `xtol_rel` or `xtol_abs` was reached.\n";
            std::cout << "EXIT: Optimal Solution Found.\n--------\n\n";
            break;
        case 5:
            std::cout << "NLOPT status: Optimization stopped because `maxeval` was reached.\n";
            std::cout << "EXIT: Optimal Solution Found.\n--------\n\n";
            break;
        case 6:
            std::cout << "NLOPT status: Optimization stopped because `maxtime` was reached.\n";
            std::cout << "EXIT: Optimal Solution Found.\n--------\n\n";
            break;
        default:
            std::cout << "result code: " << result << "\n\n";
            break;
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Initial guess as linspace from start to goal
/// @param start_state
/// @param goal_state
///////////////////////////////////////////////////////////////////////
std::vector<double> 
KOMO::SetInitialGuess(const std::vector<double> &start_state, const std::vector<double> &goal_state)
{
    auto q0 = linspace(start_state[0], goal_state[0], 20);
    auto q1 = linspace(start_state[1], goal_state[1], 20);
    auto q2 = linspace(start_state[2], goal_state[2], 20);
    auto q3 = linspace(start_state[3], goal_state[3], 20);
    auto q4 = linspace(start_state[4], goal_state[4], 20);
    auto q5 = linspace(start_state[5], goal_state[5], 20);
    auto q6 = linspace(start_state[6], goal_state[6], 20);

    std::vector<double> init;
    for (uint i=0; i<20; ++i)
    {
        init.push_back(q0[i]);
        init.push_back(q1[i]);
        init.push_back(q2[i]);
        init.push_back(q3[i]);
        init.push_back(q4[i]);
        init.push_back(q5[i]);
        init.push_back(q6[i]);
    }
    return init;
}
