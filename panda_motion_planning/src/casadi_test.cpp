#include <ros/ros.h>
#include <casadi/casadi.hpp>


#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "casadi_test");
    ros::NodeHandle nh;

    int num_states = 2;
    int num_collocation_points = 7;
    int trajectory_duration = 2;
    double delta_t = static_cast<double>(num_collocation_points)/trajectory_duration;

    // Create decision variables
    casadi::SX x = casadi::SX::sym("x", 1, 2);
    // casadi::SX u = casadi::SX::sym("u", num_collocation_points, 1);


    auto objective = pow(x(0), 2) + pow(x(1), 2);
    std::cout << objective << std::endl;


    // solver = casadi.nlpsol('solver', 'ipopt', {'x': variables_flat, 'f': objective, 'g': sys_dynamics_flat})
    // result = solver(x0=state_init, lbg=0.0, ubg=0.0,
    //                 lbx=pack_variables_fn(**lower_bounds)['flat'],
    //                 ubx=pack_variables_fn(**upper_bounds)['flat'])

    return 0;
}