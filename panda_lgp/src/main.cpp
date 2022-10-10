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

    for (auto phase : komo.phases)
    {
        std::cout << "Phase " << phase.ID << ":\t";
        for (auto g : phase.constraints)
        {
            std::cout << "constraint: " << g << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    KomoStatus komo_status = komo.Optimize(LgpLevel::THIRD_LEVEL);
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



    
    

    ros::waitForShutdown();
    return 0;
}