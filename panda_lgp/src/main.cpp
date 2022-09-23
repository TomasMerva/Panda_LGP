#include <ros/ros.h>
#include <panda_lgp/KOMO/komo.h>
#include <panda_lgp/utils/kinematics.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nlopt");
    ros::NodeHandle nh;

    kinematics::Configuration robot_config;
    KOMO komo(nh);
    komo.SetModel(robot_config);
    // num_phases, time slices, seconds for traj, k-order
    komo.SetTiming(3, 50, 5, 2);

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

   
    return 0;
}