#include <ros/ros.h>


#include <drake/solvers/mathematical_program.h>
// #include <drake/solvers/solve.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/solver_options.h>

#include <cmath>
#include <typeinfo>


using drake::solvers::MathematicalProgram;
using drake::solvers::IpoptSolver;


typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowVector;


#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ifopt_mp");
    ros::NodeHandle nh;

    MathematicalProgram program;    
    auto x = program.NewContinuousVariables(2);
    program.AddConstraint(x[0] + x[1] == 1);
    program.AddConstraint(x[0] <= x[1]);
    program.AddCost(pow(x[0],2) + pow(x[1],2));


    drake::solvers::SolverOptions options;
    options.SetOption(drake::solvers::IpoptSolver::id(), "linear_solver", "mumps");
    IpoptSolver solver;
    RowVector initial_guess(2);
    initial_guess << 1, 1;


        
    auto t1 = high_resolution_clock::now();
    auto result = solver.Solve(program, initial_guess);
    auto t2 = high_resolution_clock::now();

    std::cout << result.is_success() << "\n";
    std::cout << result.GetSolution(x) << "\n";
    std::cout << result.get_solver_id().name() << std::endl;

    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << ms_double.count() << "ms\n";

    return 0;
}