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
void KOMO::SetModel(kinematics::Configuration robot_conf, LgpLevel level)
{    
    if (level == LgpLevel::SECOND_LEVEL)
    {
        // Dim of the decision variables and their boundaries
        uint x_t_dim = robot_conf.q_dim + robot_conf.frame_dim;
        _NLP_model.x_dim = x_t_dim * phases.size();
        _NLP_model.lower_bounds = std::vector<double>(_NLP_model.x_dim);
        _NLP_model.upper_bounds = std::vector<double>(_NLP_model.x_dim);

        for (uint id = 0; id < phases.size(); ++id)
        {
            // ---- Set joint limits ----
            // q1 limit
            _NLP_model.lower_bounds[0+id*x_t_dim] = robot_conf.joint_limits.q1_limit.first;
            _NLP_model.upper_bounds[0+id*x_t_dim] = robot_conf.joint_limits.q1_limit.second;
            // q2 limit
            _NLP_model.lower_bounds[1+id*x_t_dim] = robot_conf.joint_limits.q2_limit.first;
            _NLP_model.upper_bounds[1+id*x_t_dim] = robot_conf.joint_limits.q2_limit.second;
            // q3 limit
            _NLP_model.lower_bounds[2+id*x_t_dim] = robot_conf.joint_limits.q3_limit.first;
            _NLP_model.upper_bounds[2+id*x_t_dim] = robot_conf.joint_limits.q3_limit.second;
            // q4 limit
            _NLP_model.lower_bounds[3+id*x_t_dim] = robot_conf.joint_limits.q4_limit.first;
            _NLP_model.upper_bounds[3+id*x_t_dim] = robot_conf.joint_limits.q4_limit.second;
            // q5 limit
            _NLP_model.lower_bounds[4+id*x_t_dim] = robot_conf.joint_limits.q5_limit.first;
            _NLP_model.upper_bounds[4+id*x_t_dim] = robot_conf.joint_limits.q5_limit.second;
            // q6 limit
            _NLP_model.lower_bounds[5+id*x_t_dim] = robot_conf.joint_limits.q6_limit.first;
            _NLP_model.upper_bounds[5+id*x_t_dim] = robot_conf.joint_limits.q6_limit.second;
            // q7 limit
            _NLP_model.lower_bounds[6+id*x_t_dim] = robot_conf.joint_limits.q7_limit.first;
            _NLP_model.upper_bounds[6+id*x_t_dim] = robot_conf.joint_limits.q7_limit.second;

            // ---- Set object limits ----
            
        }
    }
    else if (level == LgpLevel::THIRD_LEVEL)
    {
        // Dim of the decision variables
       _NLP_model.x_dim = (robot_conf.q_dim * phases[0].num_time_slices) * phases.size();
    }
    else
    {}

    for (auto lb : _NLP_model.lower_bounds)
    {
        std::cout << lb << "\t";
    }
    std::cout << "\n";

 //   _configuration = robot_conf;

    /*
    1. Vytvorit jeden velky vektor na zaklade info z phases pre NLOPT
    2. Vytvorit boundaries pre tento vektor
    3. Ako riesit constraints?
    */
}

///////////////////////////////////////////////////////////////////////
/// @brief Creates phases based on the parameters
/// @param num_phases Number of phases
/// @param num_time_slices Number of time slices between phases
/// @param seconds Time between each trajectory
/// @param k_order KOMO parameter for the objective function
///////////////////////////////////////////////////////////////////////
void KOMO::SetTiming(const double num_phases, const double num_time_slices, 
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
void KOMO::AddConstraint(const std::vector<uint> phases_ID, Constraint::ConstraintSymbol g)
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
void KOMO::ClearConstraint(const std::vector<uint> phases_ID)
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
void KOMO::Reset()
{
    phases.clear();
}


///////////////////////////////////////////////////////////////////////
/// @brief Find a solution
/// @param
///////////////////////////////////////////////////////////////////////
KomoStatus KOMO::Optimize(LgpLevel level)
{
    if (level == LgpLevel::SECOND_LEVEL)
    {
        uint x_dim = 7 + 3; //q(7) + cube[x,y,z]

        // 1. Choose solver and set decision variables
        nlopt::opt opt(nlopt::algorithm::LD_AUGLAG, phases.size()*x_dim);
        nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, phases.size()*x_dim);
        opt.set_xtol_abs(1e-6);
        opt.set_local_optimizer(local_opt);

        // 2. Set boundaries
        // - convert boundaries from all phases into one vector
        std::vector<double> lower_bounds;
        std::vector<double> upper_bounds;
        for (auto phase : phases)
        {
            lower_bounds.insert(lower_bounds.begin(), phase.lower_bounds.begin(), phase.lower_bounds.end());
            upper_bounds.insert(upper_bounds.begin(), phase.upper_bounds.begin(), phase.upper_bounds.end());
        }
        opt.set_lower_bounds(lower_bounds);
        opt.set_upper_bounds(upper_bounds);

        // 3. Set objective function
        KOMO_k2::ObjectiveData objective;
        objective.num_phase_variables = x_dim;
        objective.num_phases = phases.size();
        objective.num_iterations = 0;
        opt.set_min_objective(KOMO_k2::GetCost, NULL);

        // 4. Set constraints
        // TODO: 

        // 5. Set an initial guess
        std::vector<double> x;
        for (auto phase : phases)
        {
            std::cout << "phase size: " << phase.x_init.size() << std::endl;
            x.insert(x.begin(), phase.x_init.begin(), phase.x_init.end());
        }
        std::cout << "x size: " << x.size() << std::endl;
        
        // 6. Optimize
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
            switch (result)
            {
                // Positive messages
                case 1:
                    std::cout << "NLOPT status: Generic success return value.\n";
                    std::cout << "EXIT: Optimal Solution Found.\n--------\n";
                    break;
                case 2:
                    std::cout << "NLOPT status: Optimization stopped because `stopval` was reached.\n";
                    std::cout << "EXIT: Optimal Solution Found.\n--------\n";
                    break;
                case 3:
                    std::cout << "NLOPT status: Optimization stopped because `ftol_rel` or `ftol_abs` was reached.\n";
                    std::cout << "EXIT: Optimal Solution Found.\n--------\n";
                    break;
                case 4:
                    std::cout << "NLOPT status: Optimization stopped because `xtol_rel` or `xtol_abs` was reached.\n";
                    std::cout << "EXIT: Optimal Solution Found.\n--------\n";
                    break;
                case 5:
                    std::cout << "NLOPT status: Optimization stopped because `maxeval` was reached.\n";
                    std::cout << "EXIT: Optimal Solution Found.\n--------\n";
                    break;
                case 6:
                    std::cout << "NLOPT status: Optimization stopped because `maxtime` was reached.\n";
                    std::cout << "EXIT: Optimal Solution Found.\n--------\n";
                    break;
                default:
                    break;
            }
        }
        catch(std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }


    }
    else if(level == LgpLevel::THIRD_LEVEL)
    {

    }
    else
    {}
    

    

    // 3. Set objective


    // 4. Solve



    // if (level == LgpLevel::SECOND_LEVEL)
    // {
    //     // nlopt::opt opt(nlopt::algorithm::LD_AUGLAG, x->GetNumVariables());
    //     // nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, x->GetNumVariables());
    //     // opt.set_xtol_abs(1e-6);
    //     // opt.set_local_optimizer(local_opt);
    // }
    // else if (level == LgpLevel::THIRD_LEVEL)
    // {
    //     // nlopt::opt opt(nlopt::algorithm::LD_AUGLAG, x->GetNumVariables());
    //     // nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, x->GetNumVariables());
    //     // opt.set_xtol_abs(1e-6);
    //     // opt.set_local_optimizer(local_opt);
    // }
    // else
    // {}
}


void KOMO::AddJointLimits(std::vector<double> &lower_bounds, std::vector<double> &upper_bounds)
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
