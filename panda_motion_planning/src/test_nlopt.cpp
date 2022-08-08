#include <ros/ros.h>
#include <nlopt.hpp>
#include <iomanip>
#include <Eigen/Dense>
#include <panda_motion_planning/constraints/constraints_list.h>

// typedef struct {
//     double a;
//     double b;
//     size_t idx;
//     double tolerance;
// } my_constraint_data;

double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    if (!grad.empty()) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nlopt");
    ros::NodeHandle nh;

    nlopt::opt opt(nlopt::LD_MMA, 2);
    std::vector<double> lb(2);
    lb[0] = -HUGE_VAL; lb[1] = 0;
    opt.set_lower_bounds(lb);
    opt.set_min_objective(myvfunc, NULL);


    // ConstraintData data[2];
    // data[0] = {2, 0, 1, 0.001, 1, 1, 1};
    // data[1] = {-1, 1, 1, 0.001, 1, 1, 1};
    // opt.add_inequality_constraint(Constraints::myvconstraint, &data[0], 1e-8);
    // opt.add_inequality_constraint(Constraints::myvconstraint, &data[1], 1e-8);
    
    opt.set_xtol_rel(1e-4);
    std::vector<double> x(2);
    x[0] = 1.234; x[1] = 5.678;
    double minf;

    try{
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
            << std::setprecision(10) << minf << std::endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }


    return 0;
}