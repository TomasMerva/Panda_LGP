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
    // nlopt::algorithm::LD_AUGLAG_EQ AUGLAG
    nlopt::opt opt(nlopt::algorithm::AUGLAG, x->GetNumVariables());
    nlopt::opt local_opt(nlopt::algorithm::LD_LBFGS, x->GetNumVariables());
    // nlopt::opt local_opt(nlopt::algorithm::LD_SLSQP, x->GetNumVariables());
    //LD_SLSQP LD_MMA LD_TNEWTON_PRECOND LD_LBFGS

    local_opt.set_xtol_abs(0.01);
    opt.set_local_optimizer(local_opt);
    opt.set_maxtime(60);
    opt.set_xtol_abs(0.0001);
    
    // 2. set bounds
    x->SetBoundaryConstraints(_start_state, _goal_state);
    opt.set_lower_bounds(x->GetLowerBounds());
    opt.set_upper_bounds(x->GetUpperBounds());

    // 3. set an objective function
    opt.set_min_objective(objective->GetCost, NULL);


    // 4. set constraints
    AddPointToPointDistanceData constraint_data[20];
    std::vector<nlopt::vfunc> nlopt_constraints;

    for(size_t i =0; i< _num_time_slices; i++)
    {
        nlopt_constraints.push_back([](const std::vector<double>& x, std::vector<double>& grad, void* data) -> double {
            AddPointToPointDistanceData *d = reinterpret_cast<AddPointToPointDistanceData*>(data);
            int num_variables = 7;
            int idx = d->idx; 
            Eigen::VectorXd obj_pos(3);
            obj_pos << d->obj_pos_x, d->obj_pos_y, d->obj_pos_z;
            double tolerance = d->tolerance;
            double delta_q = 0.001;

            Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*num_variables, num_variables);

            auto FK_q = Kinematics::ForwardKinematics(q, true);
            Eigen::VectorXd pos_t(3);
            pos_t << FK_q(0,3), FK_q(1,3), FK_q(2,3);
            auto diff_pos = obj_pos - pos_t;
            double l2_norm = - sqrt((diff_pos.transpose()*diff_pos)[0]) + 0.3;

            if (!grad.empty())
            {
                std::fill(grad.begin(), grad.end(), 0);
                Eigen::MatrixXd J(3, num_variables);
                for (int i=0; i<num_variables; i++)
                {
                    auto temp = q;
                    temp(i) += delta_q; // ad delta_q to the q(i)
                    auto FK_with_deltaq = Kinematics::ForwardKinematics(temp, true); //compute FK with q(i)+delta_q
                    J(0, i) = (FK_with_deltaq(0,3) - FK_q(0,3)) / delta_q;  // dx / dq(i)
                    J(1, i) = (FK_with_deltaq(1,3) - FK_q(1,3)) / delta_q;  // dy / dq(i)
                    J(2, i) = (FK_with_deltaq(2,3) - FK_q(2,3)) / delta_q;  // dz / dq(i)
                }
                auto dg = diff_pos.transpose()*J;
                for (int i=0; i<num_variables; i++)
                {
                    grad[i + idx*num_variables] = dg[i];
                }
                
            }
            return l2_norm;
        });
    }

    const double kTolerance = 1e-4;
    for (int i=0; i<_num_time_slices; i++)
    {
        constraint_data[i] = {i, 0.5, 0.0, 0.1, 0.3};
        opt.add_inequality_constraint(nlopt_constraints[i], &constraint_data[i], kTolerance);
    }


    // opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[0], 1e-10);
    // opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[1], 1e-10);
    // opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[2], 1e-10);
    // opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[3], 1e-10);
    // opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[4], 1e-10);


    // AddPointToPointDistanceData constraint_data[_num_time_slices];
    // for (int timestep=1; timestep < (_num_time_slices-1); timestep++)
    // {
    //     constraint_data[timestep-1] = {timestep, 0.5, 0.0, 0.1, 0.3};
    //     opt.add_inequality_constraint(Constraint::AddPointToPointDistanceConstraint, &constraint_data[timestep-1], 1e-10);
    // }
    
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
        std::cout << "------------------------------------------\n";
        duration<double, std::milli> ms_double = finish_time - start_time;
        std::cout << "Number of iterations: \t=\t" << objective->num_iterations << "\n";
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
            // Negative messages
            // case -1:
            //     std::cout << "NLOPT status: Generic failure code.\n";
            //     std::cout << "EXIT: Optimal Solution cannot be found.\n--------\n"; 
            //     break;
            // case -2:
            //     std::cout << "NLOPT status: Invalid arguments (e.g. lower bounds are bigger than upper bounds, an unknown algorithm was specified, etcetera).\n";
            //     std::cout << "EXIT: Optimal Solution cannot be found.\n--------\n";
            //     break;
            // case -3:
            //     std::cout << "NLOPT status: Ran out of memory.\n";
            //     std::cout << "EXIT: Optimal Solution cannot be found.\n--------\n";
            //     break;
            // case -4:
            //     std::cout << "NLOPT status: Halted because roundoff errors limited progress. (In this case, the optimization still typically returns a useful result.)\n";
            //     std::cout << "EXIT: Optimal Solution cannot be found.\n--------\n";
            //     break;
            // case -5:
            //     std::cout << "NLOPT status: Halted because of a forced termination: The user called a stop function\n";
            //     std::cout << "EXIT: Optimal Solution cannot be found.\n--------\n";
            //     break;
            default:
                break;
        }       
    }
    catch(std::exception &e) 
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;         
    }

    // TODO : ADD success termination


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