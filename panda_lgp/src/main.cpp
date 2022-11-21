#include <ros/ros.h>
#include <panda_lgp/KOMO/komo.h>
#include <panda_lgp/utils/kinematics.h>

#include <panda_lgp/action_skeleton/operators.h>
#include <panda_lgp/action_skeleton/skeleton.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nlopt");
    ros::NodeHandle nh;
    
    // ROS init
    ros::AsyncSpinner spinner(1);
    spinner.start();

    KOMO komo(nh);
    ros::Duration(0.1).sleep();
    // num_phases, time slices, seconds for traj, k-order
    // komo.SetTiming(4, 20, 5, 2);

    logic::Skeleton S({
        logic::SkeletonEntry(logic::SkeletonAction::MoveF, {"panda_link8", "red_region", "grey_region"}),
        logic::SkeletonEntry(logic::SkeletonAction::Pick, {"panda_link8", "cube_A", "grey_region"} ),
        logic::SkeletonEntry(logic::SkeletonAction::MoveH, {"panda_link8", "cube_A", "grey_region", "red_region"} ),
        logic::SkeletonEntry(logic::SkeletonAction::Place, {"panda_link8", "cube_A", "red_region"} )
    });
    // logic::Skeleton S({
    //     logic::SkeletonEntry(logic::SkeletonAction::MoveF, {"panda_link8", "red_region", "grey_region"})    
    // });

    S.SetKOMO(&komo);
    // std::cout << komo << std::endl;

    // KomoStatus komo_status = komo.Optimize(LgpLevel::SECOND_LEVEL);
    // if (komo_status == KomoStatus::KS_SolutionFound)
    // {
    //     for (auto const phase : komo.phases)
    //     {
    //         std::cout << "----- Action:  " << phase.symbolic_name << " -----\n[";
    //         for (auto const x_phase : phase.x)
    //         {
    //             std::cout << x_phase << "      ";
    //         }
    //         std::cout << "]\n";
    //         auto FK = kinematics::ForwardKinematics(Eigen::Map<const Eigen::VectorXd>(phase.x.data(), phase.x.size()), true);
    //         std::cout << "EEF[x,y,z] = [" << FK(0,3) << " " << FK(1,3) << " " << FK(2,3) << "]\n\n";
    //     }
    // }
    // else
    // {
    //     std::cout << "No solution has been found" << std::endl;
    // }

    // just to test 3.level
    komo.phases[1].x = std::vector<double>{-1.2116, 0.356153, 0.244033, -2.28525, -1.68591, 2.35773, 4.035e-08};
    komo.phases[2].x = std::vector<double>{-1.06086, 0.436372, 0.461309, -2.32378, -1.67167, 2.46085, 4.035e-08};
    komo.phases[3].x = std::vector<double>{-0.747309, 0.356153, 0.908409, -2.42052, -1.6404, 2.67201, 4.035e-08};
    komo.phases[4].x = std::vector<double>{-0.747309, 0.5, 0.908409, -2.42052, -1.6404, 2.67201, 4.035e-08};    
    KomoStatus komo_status = komo.Optimize(LgpLevel::THIRD_LEVEL);
    if (komo_status == KomoStatus::KS_SolutionFound)
    {
        ros::Duration(5).sleep();

        std_msgs::ColorRGBA marker_color;
        marker_color.g = 1;
        marker_color.a = 1;
        komo.VisualizeTrajectory(komo.phases[0].q_trajectory, marker_color, 0);

        marker_color.g = 0;
        marker_color.r = 1;
        marker_color.a = 1;
        komo.VisualizeTrajectory(komo.phases[1].q_trajectory, marker_color, 1);

        marker_color.g = 0;
        marker_color.b = 1;
        marker_color.r = 0;
        marker_color.a = 1;
        komo.VisualizeTrajectory(komo.phases[2].q_trajectory, marker_color, 2);

        marker_color.g = 1;
        marker_color.b = 1;
        marker_color.r = 0;
        marker_color.a = 1;
        komo.VisualizeTrajectory(komo.phases[3].q_trajectory, marker_color, 3);
    }
    else 
    {
        ROS_ERROR("No solution has been found");
    }




    // const std::vector<double> x{-1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08, 
    //                             -0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08,
    //                             0.096, 0.308, 0.646, -2.249, 0.06, 2.53, 2.019};
    // //                             0.596, 0., 0., -2, 0.06, 1.53, 1.019,
    // //                             -1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08,
    // //                              -0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08};
    // std::vector<double> jac(x.size());
    // KOMO_k2::FillJacobianBlock(x, jac, 7, 6);

    // for (auto g : jac)
    // {
    //     std::cout << g << " ";
    // }
    // std::cout << "\n";

    // const std::vector<double> x{-1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08, 0, 2, 0, 0, 0, 0};
    // std::vector<double> grad(x.size());
    // Constraint::ConstraintData g_data;
    // g_data.num_phase_variables = x.size();
    // g_data.idx = 0;
    // std::vector<double> grey_region = {-0.5, 0.2};
    // std::vector<double> red_region = {0.2, 0.5};
    // g_data.region = grey_region;
    // auto g = Constraint::AxisInRegion(x, grad, &g_data);
    // std::cout << g << "\n";

    // for (auto gr : grad)
    // {
    //     std::cout << gr << " ";
    // }
    // std::cout << "\n";

    // uint m = 1;
    // uint n = 13*4;
    // double x[n] = {-1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08, 0, 0, 0, 0, 0, 0,
    //                 -0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08, 0, 0, 0, 0, 0, 0,
    //                 0.096,   0.308,    0.646,    -2.249,   0.06,     2.53,    2.019,        0, 0, 0, 0, 0, 0,
    //                 0.596,   0.,       0.,       -2,       0.06,     1.53,    1.019,        0, 0, 0, 0, 0, 0};
    // double grad[n*m];

    // Constraint::ConstraintData g_data;
    // g_data.idx=0;
    // g_data.num_phase_variables = 13;
    // g_data.region = std::vector<double>{-0.5, 0.2};
   
    // double result[m];
    // Constraint::AxisInRegion(m, &result[0], n, &x[0], &grad[0], &g_data);
    // for (uint i=0; i<2; i++)
    // {
    //     std::cout << result[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "\ngrad" << std::endl;

    // for (uint i=0; i<m*n; ++i)
    // {
    //     std::cout << grad[i] << " ";
    //     if (i==n) std::cout << "\n\n";
    // }
    // std::cout << std::endl;


    // uint timesteps = 4;
    // uint m = timesteps;
    // uint n = 7*timesteps;
    // double x[n] = {-1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08,
    //                 -0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08,
    //                 -0.747309, 0.906277, 0.908409, -2.42052, -1.6404, 2.67201, 4.035e-08,
    //                 -0.507423, 1.26222, 1.34696, -2.40899, -1.60368, 2.87996, 4.035e-08};
    // double grad[n*m];
    // const std::vector<double> k_tolerance(timesteps, 1e-6);
    // Constraint::ConstraintData g_data;
    // g_data.num_phase_variables = 7;
    // double result[m];


    // auto t1 = high_resolution_clock::now();
    // Constraint::Zaxis(m, &result[0], n, &x[0], &grad[0], &g_data);
    // auto t2 = high_resolution_clock::now();
    // duration<double, std::milli> ms_double = t2 - t1;
    // std::cout << ms_double.count() << "ms\n";

    // for (uint i=0; i<timesteps; i++)
    // {
    //     std::cout << result[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "\ngrad" << std::endl;

    // for (uint i=0; i<m*n; ++i)
    // {
    //     std::cout << grad[i] << " ";
    //     if (i%n==0 && i>0) 
    //     {
    //         std::cout << "\n\n";
    //     }
    // }
    // std::cout << std::endl;

    ros::waitForShutdown();
    return 0;
}