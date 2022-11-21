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
    const double tolerance_solution = 1e-3;
    const double tolerance_constraints = 1e-2;
    const int max_iterations = 1000;   
    const double mu_init = 2.0;

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
        _ceresProblem->AddResidualBlock(cost_function, new ceres::ScaledLoss(nullptr, 10.0, ceres::DO_NOT_TAKE_OWNERSHIP), x.col(t).data(), x.col(t+1).data(), x.col(t+2).data());
        // _ceresProblem->AddResidualBlock(cost_function, nullptr, x.col(t).data(), x.col(t+1).data(), x.col(t+2).data());
    }

    // 5. Constraints
    std::vector<ceres::ResidualBlockId> g_id_vector;
    for (uint t=0; t<_num_timesteps; ++t)
    {
        ceres::CostFunction* g_spm_function = new Collision_FeatureCost(std::vector<double>{1.0, 0.0, 0.5});
        ceres::LossFunction* mu_loss = new MuLoss(nullptr, mu_init, ceres::DO_NOT_TAKE_OWNERSHIP);
        auto g_id = _ceresProblem->AddResidualBlock(g_spm_function, mu_loss, x.col(t).data());
        g_id_vector.push_back(g_id);

        ceres::CostFunction* g_lagrangian_function = new Collision_Langrange(t, std::vector<double>{1.0, 0.0, 0.5});
        _ceresProblem->AddResidualBlock(g_lagrangian_function, new ceres::ScaledLoss(nullptr, 2.0, ceres::DO_NOT_TAKE_OWNERSHIP), x.col(t).data());
    }    


    // 6. Set solver parameters
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = max_iterations;
    options.parameter_tolerance = 10*tolerance_solution;
    ceres::Solver::Summary summary;


    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    auto t1 = high_resolution_clock::now();

    auto x_old = x;
    size_t counter = 1;
    const size_t max_iterations_outer_loop = 100;
    while(true)
    {
        // Unconstrained optimization using Ceres
        ceres::Solve(options, _ceresProblem, &summary);
        // std::cout << summary.FullReport() << "\n";
        std::cout << "--- Iteration: " << counter << "  ---\tmu: " << MuLoss::nu << "\n";
        std::cout << Eigen::Map<Eigen::VectorXd>(Collision_Langrange::lambda.data(), Collision_Langrange::lambda.size()) << "\n";
        

        auto diff = x_old - x;
        // evaluate constraints
        Eigen::ArrayXXd g_values(1, _num_timesteps);
        for (uint t=0; t<_num_timesteps; ++t)
        {
            _ceresProblem->EvaluateResidualBlock(g_id_vector[t], false, g_values.col(t).data(), nullptr, nullptr);
        } 

        // Stopping criteria
        bool x_tol = (diff.abs() <= tolerance_solution).all();
        bool g_tol = (g_values <= tolerance_constraints).all();
        bool iter_tol = counter >= max_iterations_outer_loop;
        if ((x_tol && g_tol) || iter_tol)
        {
            if (g_tol) std::cout << "Constraint tolerance reached. Optimal solution found...\n";
            else if (iter_tol) std::cout << "Max iteration reached. Cannot find solution...\n";
            break;
        }

        //---- Update ----
        for (uint t=0; t<_num_timesteps; ++t)
        {
            Collision_Langrange::lambda[t] = std::max(Collision_Langrange::lambda[t] + 2*MuLoss::nu*g_values(0, t), 0.0);
        }
        MuLoss::nu *=10;
        counter++;
        x_old = x;
    }
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << ms_double.count() << "ms\n";
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