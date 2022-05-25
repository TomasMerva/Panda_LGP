#include <ros/ros.h>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <panda_motion_planning/simple_optim_mp.h>

#include <drake/solvers/mathematical_program.h>
// #include <drake/solvers/solve.h>
#include <drake/solvers/ipopt_solver.h>

#include <cmath>
#include <typeinfo>

using namespace ifopt;

using drake::solvers::MathematicalProgram;
using drake::solvers;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ifopt_mp");
    ros::NodeHandle nh;

    int num_states = 3;
    int num_collocation_points = 10;

    MathematicalProgram program;
    // auto x = program.NewContinuousVariables(num_collocation_points, num_states, "joints");
    
    auto x = program.NewContinuousVariables(2);
    program.AddConstraint(x[0] + x[1] == 1);
    program.AddConstraint(x[0] <= x[1]);
    program.AddCost(pow(x[0],2) + pow(x[1],2));

    IpoptSolver solver;
    Eigen::VectorXd initial_guess;
    initial_guess << 1, 1;
    auto result = solver.Solve(program, initial_guess, 0);

    


    // std::cout << result.is_success() << "\n";
    // std::cout << result.GetSolution(x) << "\n";
    // std::cout << result.get_solver_id().name() << std::endl;


    return 0;
}