#include <ros/ros.h>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <nlopt.hpp>
// #include <nlopt.h>
#include <math.h>

#include <panda_motion_planning/komo_lib.h>
#include <panda_motion_planning/variables/joints.h>
#include <panda_motion_planning/objective/komo_k2.h>

#include "gnuplot-iostream.h"


template<typename T>
void PlotData(const std::vector<T> x_data, const std::vector<T> y_data, const std::string label)
{
    std::vector<std::pair<T, T>> plot_vector(x_data.size());
    for (size_t i=0; i<plot_vector.size(); i++)
    {
      plot_vector[i] = std::make_pair(x_data[i], y_data[i]);
    }

    Gnuplot gp;
    // gp << "set multiplot layout 2,2\n";
    gp << "set style line 1 \
      linecolor rgb '#0060ad' \
      linetype 1 linewidth 2 \
      pointtype 7 pointsize 1.5\n";
    gp << "plot" << gp.file1d(plot_vector) << "with linespoints linestyle 1 title '" << label << "'\n";

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "komo");
    ros::NodeHandle nh;

    const int num_joints = 7;
    const int num_time_slices = 20;
    std::vector<double> q_start{0.0, -0.7856, -1.4014, -2.3559, -1.1646, 1.5717, 0.7853};
    std::vector<double> q_goal{-0.2919, 0.1937, -0.1681, -2.086, 0.0369, 2.3322, 0.3197};

    JointsVariable x(num_joints, num_time_slices);
    KOMO_k2 objective(num_joints, num_time_slices);

    // 1. choose solvers
    nlopt::opt opt(nlopt::algorithm::AUGLAG, x.GetNumVariables());
    nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND, x.GetNumVariables());
    local_opt.set_xtol_abs(0.01);
    opt.set_xtol_abs(0.001);
    opt.set_local_optimizer(local_opt);

    // 2. set bounds
    x.SetBoundaryConstraints(q_start, q_goal);
    opt.set_lower_bounds(x.GetLowerBounds());
    opt.set_upper_bounds(x.GetUpperBounds());
    

    // 3. set objective
    //TODO: spytaj sa laciho preco to musim dat ako static
    opt.set_min_objective(objective.GetCost, NULL);

    // 4. set constraints
    // TODO: not yet
    
    // 5. Set initial guess
    std::vector<double> q = x.InitialGuess(q_start, q_goal);


    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;


    // 6. optimize
    double minf;    // the minimum objective value upon return
    
    auto t1 = high_resolution_clock::now();
    try{
        nlopt::result result = opt.optimize(q, minf);
        std::cout << "found minimum at f(" << minf << std::endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << ms_double.count() << "ms\n";


    std::vector<std::vector<double>> result;
    int idx =0;
    for (int col=0; col<num_time_slices; col++)
    {
        std::vector<double> temp;
        for (int row=0; row<num_joints; row++)
        {
            temp.push_back(q[idx]);
            idx++;
        }
        result.push_back(temp);
    }

    for (int row=0; row<num_joints; row++)
    {
        for (int col=0; col<num_time_slices; col++)
        {
            std::cout << result[col][row] << " ";
        }
        std::cout << std::endl;
    }


    std::vector<double> time = linspace(0, 5, num_time_slices);
    std::vector<double> q1, q2, q3, q4, q5, q6, q7;
    for (int i=0; i<num_time_slices; i++)
    {
        q1.push_back(result[i][0]);
        q2.push_back(result[i][1]);
        q3.push_back(result[i][2]);
        q4.push_back(result[i][3]);
        q5.push_back(result[i][4]);
        q6.push_back(result[i][5]);
        q7.push_back(result[i][6]);
    }
    // PlotData(time, q1, "joint1");
    PlotData(time, q2, "joint2");
    // PlotData(time, q3, "joint3");
    // PlotData(time, q4, "joint4");
    // PlotData(time, q5, "joint5");
    // PlotData(time, q6, "joint6");
    // PlotData(time, q7, "joint7");


    
    return 0;
}