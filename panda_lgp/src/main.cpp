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




    // komo.AddConstraint({1, 2}, ConstraintSymbol::FS_PointToPointDistance);
    // komo.AddConstraint({2}, ConstraintSymbol::FS_FixedOrientationAxis);
    // komo.ClearConstraint({1});
    // // TODO: AddObjective(const std::vector<uint> phases_ID, Objective_ENUM)


    // for (auto phase : komo.phases)
    // {
    //     std::cout << "Phase " << phase.ID << ":\t";
    //     for (auto g : phase.constraints)
    //     {
    //         std::cout << "constraint: " << g << "\t";
    //     }
    //     std::cout << "\n";
    // }
    // std::cout << "\n";
    //     // komo.SetModel(robot_config, LgpLevel::SECOND_LEVEL);

    // logic::Skeleton S({
    //     logic::SkeletonEntry(logic::SkeletonAction::MoveF, {"panda_link8", "red_region", "grey_region"}),
    //     logic::SkeletonEntry(logic::SkeletonAction::Pick, {"panda_link8", "cube_A", "grey_region"} ),
    //     logic::SkeletonEntry(logic::SkeletonAction::MoveH, {"panda_link8", "cube_A", "grey_region", "red_region"} ),
    //     logic::SkeletonEntry(logic::SkeletonAction::Place, {"panda_link8", "cube_A", "red_region"} )
    // });
    logic::Skeleton S({
        logic::SkeletonEntry(logic::SkeletonAction::MoveF, {"panda_link8", "red_region", "grey_region"}),
        logic::SkeletonEntry(logic::SkeletonAction::MoveH, {"panda_link8", "cube_A", "grey_region", "red_region"} ),
    });

    S.SetKOMO(&komo);
    KomoStatus komo_status = komo.Optimize(LgpLevel::SECOND_LEVEL);



    if (komo_status == KomoStatus::KS_SolutionFound)
    {
        for (auto const phase : komo.phases)
        {
            std::cout << "----- Action:  " << phase.symbolic_name << " -----\n[";
            for (auto const x_phase : phase.x)
            {
                std::cout << x_phase << "      ";
            }
            std::cout << "]\n";
            auto FK = kinematics::ForwardKinematics(Eigen::Map<const Eigen::VectorXd>(phase.x.data(), phase.x.size()), true);
            std::cout << "EEF[x,y,z] = [" << FK(0,3) << " " << FK(1,3) << " " << FK(2,3) << "]\n\n";
        }
    }
    else
    {
        std::cout << "No solution has been found" << std::endl;
    }



    // // // just to test 3.level
    // komo.phases[1].x = std::vector<double>{-1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08, 0, 0, 0, 0, 0, 0};
    // komo.phases[2].x = std::vector<double>{-0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08, 0, 0, 0, 0, 0, 0};
    // KomoStatus komo_status = komo.Optimize(LgpLevel::THIRD_LEVEL);
    // if (komo_status == KomoStatus::KS_SolutionFound)
    // {
    //     for (auto const phase : komo.phases)
    //     {
    //         for (auto t_q : phase.q_trajectory)
    //         {
    //             for (auto q: t_q)
    //             {
    //                 std::cout << q << "   ";
    //             }
    //             std::cout << "\n\n";
    //         }
    //     }
    // }




    // const std::vector<double> x{-1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08, 
    //                             -0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08,
    //                             0.096, 0.308, 0.646, -2.249, 0.06, 2.53, 2.019,
    //                             0.596, 0., 0., -2, 0.06, 1.53, 1.019,
    //                             -1.01003, 0.619206, -0.0457302, -2.07182, -1.69616, 2.20528, -8.00068e-08,
    //                              -0.490178, 1.10071, 0.748854, -2.19579, -1.67757, 2.41694, -8.00068e-08};
    // std::vector<double> jac(x.size());
    // KOMO_k2::FillJacobianBlock(x, jac, 7, 6);

    // for (auto g : jac)
    // {
    //     std::cout << g << " ";
    // }
    // std::cout << "\n";


    

    ros::waitForShutdown();
    return 0;
}