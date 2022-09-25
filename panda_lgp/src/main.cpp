#include <ros/ros.h>
#include <panda_lgp/KOMO/komo.h>
#include <panda_lgp/utils/kinematics.h>

#include <panda_lgp/logic/operators.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nlopt");
    ros::NodeHandle nh;

    kinematics::Configuration robot_config;
    KOMO komo(nh);
    // num_phases, time slices, seconds for traj, k-order
    komo.SetTiming(4, 20, 5, 2);




    komo.AddConstraint({1, 2}, FeatureSymbol::FS_PointToPointDistance);
    komo.AddConstraint({2}, FeatureSymbol::FS_FixedOrientationAxis);
    komo.ClearConstraint({1});
    // TODO: AddObjective(const std::vector<uint> phases_ID, Objective_ENUM)


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
        // komo.SetModel(robot_config, LgpLevel::SECOND_LEVEL);

    logic::Skeleton S({
        logic::SkeletonEntry(logic::SkeletonAction::MoveF, {"panda_link8", "red_region", "grey_region"}),
        logic::SkeletonEntry(logic::SkeletonAction::Pick, {"panda_link8", "cube_A", "grey_region"} ),
        logic::SkeletonEntry(logic::SkeletonAction::MoveH, {"panda_link8", "cube_A", "grey_region", "red_region"} ),
        logic::SkeletonEntry(logic::SkeletonAction::Place, {"panda_link8", "cube_A", "red_region"} )
    });

    for (auto SE : S.operators)
    {
        std::cout << "";
    }    

    return 0;
}