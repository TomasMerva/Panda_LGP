#include "ceres_testy/solver.h"

Solver::Solver(const int num_timestep_var, const int num_timesteps)
    : _num_timestep_var(num_timestep_var)
     ,_num_timesteps(num_timesteps)
{
    x = Eigen::ArrayXXd(_num_timestep_var, _num_timesteps);
}

void 
Solver::Solve()
{
    assert(q_start.size() == _num_timestep_var);
    assert(_num_timesteps != 0);

    // ---- Parameters ----
    const double tolerance_solution = 1e-6;
    const double tolerance_constraints = 1e-6;
    const int max_iterations = 1000;   
    const double mu_init = 1.0;

    // ---- Ceres ----
    _ceresProblem = new ceres::Problem;

    // 1. Init guess (this also creates variable representing decision variable)
    CreateInitGuess(false);

    // 2. Create parameter blocks
    for (uint t=0; t<_num_timesteps; ++t)
    {
        _ceresProblem->AddParameterBlock(x.col(t).data(), _num_timestep_var);
    }

    // 3. Boundaries #TODO: add joint limits
    SetBoundaries();

    // 4. Add main objective (KOMO)
    for (uint t=0; t<(_num_timesteps-2); ++t)
    {
        ceres::CostFunction* cost_function = new KOMO_FeatureCost;
        _ceresProblem->AddResidualBlock(cost_function, new ceres::ScaledLoss(nullptr, 2.0, ceres::DO_NOT_TAKE_OWNERSHIP), x.col(t).data(), x.col(t+1).data(), x.col(t+2).data());
    }

    // 5. Constraints
    for (uint t=0; t<_num_timesteps; ++t)
    {
        ceres::CostFunction* cost_function = new Collision_FeatureCost(std::vector<double>{1.0, 0.0, 0.5});
        ceres::LossFunction* mu_loss = new MuLoss(nullptr, 2*mu_init, ceres::DO_NOT_TAKE_OWNERSHIP);
        _ceresProblem->AddResidualBlock(cost_function, mu_loss, x.col(t).data());
    }    


    // 6. Set solver parameters
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = max_iterations;
    options.parameter_tolerance = 100*tolerance_solution;
    ceres::Solver::Summary summary;

    auto x_old = x;
    size_t counter = 1;
    while(true)
    {
        
        // Unconstrained optimization using Ceres
        ceres::Solve(options, _ceresProblem, &summary);
        std::cout << summary.FullReport() << "\n";
        std::cout << "--- Iteration: " << counter << "  ---\n";
        

        auto diff = x_old - x;
        if ( (diff.abs() <= tolerance_solution).all() )
        {
            std::cout << "finished\n";
            break;
        }
        //---- Update ----
        // MuLoss::nu *=10;
        counter++;
        x_old = x;
    }

}

void 
Solver::UnconstrainedOptimization()
{


    // assert(q_start.size() == _num_timestep_var);
    // assert(_num_timesteps != 0);

    // ceres::Problem problem;
    // for (uint t=0; t<_num_timesteps; ++t)
    // {
    //     problem.AddParameterBlock(x[t], _num_timestep_var);
    // }
    // // Boundaries for init and end state
    // for (uint i=0; i<_num_timestep_var; ++i)
    // {
    //     // Init state boundaries
    //     problem.SetParameterLowerBound(x[0], i, q_start[i]-0.0000001);
    //     problem.SetParameterUpperBound(x[0], i, q_start[i]+0.0000001);
    //      // End state boundaries
    //     problem.SetParameterLowerBound(x[_num_timesteps-1], i, q_goal[i]-0.0000001);
    //     problem.SetParameterUpperBound(x[_num_timesteps-1], i, q_goal[i]+0.0000001);
    // }

    // // Add KOMO objective
    // for (uint t=0; t<(_num_timesteps-2); ++t)
    // {
    //     ceres::CostFunction* cost_function = new KOMO_FeatureCost;
    //     problem.AddResidualBlock(cost_function, nullptr, x[t], x[t+1], x[t+2]);
    // }

    // // Add constraint
    // for (uint t=0; t<_num_timesteps; ++t)
    // {
    //     ceres::CostFunction* cost_function = new Collision_FeatureCost(std::vector<double>{1, 0, 0.5});
    //     problem.AddResidualBlock(cost_function, nullptr, x[t]);
    // }    
}


void 
Solver::CreateInitGuess(bool linspace_flag)
{
    x = Eigen::ArrayXXd(_num_timestep_var, _num_timesteps); // zero new array
    if (linspace_flag)
    {
        auto x_init = linspace(q_start[0], q_goal[0], _num_timesteps);
        auto y_init = linspace(q_start[1], q_goal[1], _num_timesteps);
        auto z_init = linspace(q_start[2], q_goal[2], _num_timesteps);
        for (uint timestep=0; timestep<_num_timesteps; ++timestep)
        {
            x.col(timestep) << x_init[timestep], y_init[timestep], z_init[timestep];
        }
    }
    else
    {
        for (uint timestep=0; timestep<_num_timesteps; ++timestep)
        {
            x.col(timestep) << q_start[0], q_start[1], q_start[2];
        }
    }
}

void
Solver::SetBoundaries()
{
    // Boundaries for init and end state
    for (uint i=0; i<_num_timestep_var; ++i)
    {
        // Init state boundaries
        _ceresProblem->SetParameterLowerBound(x.col(0).data(), i, q_start[i]-0.0000001);
        _ceresProblem->SetParameterUpperBound(x.col(0).data(), i, q_start[i]+0.0000001);
         // End state boundaries
        _ceresProblem->SetParameterLowerBound(x.col(_num_timesteps-1).data(), i, q_goal[i]-0.0000001);
        _ceresProblem->SetParameterUpperBound(x.col(_num_timesteps-1).data(), i, q_goal[i]+0.0000001);
    }
    //TODO: add joint limits
}