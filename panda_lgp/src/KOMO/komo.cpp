#include <panda_lgp/KOMO/komo.h>

///////////////////////////////////////////////////////////////////////
/// @brief Initialize publishers for RViz and franka controllers
/// @param nh ROS NodeHandle for creating publishers
///////////////////////////////////////////////////////////////////////
KOMO::KOMO(ros::NodeHandle &nh) 
    : MotionROSTools(nh)
{
}

///////////////////////////////////////////////////////////////////////
/// @brief Frankly, have no idea if this is needed
/// @param robot_conf Structure representing the robot conf, joint limits, ...
///////////////////////////////////////////////////////////////////////
void KOMO::SetModel(kinematics::Configuration robot_conf)
{
    _configuration = robot_conf;
}

///////////////////////////////////////////////////////////////////////
/// @brief Creates phases based on the parameters
/// @param num_phases Number of phases
/// @param num_time_slices Number of time slices between phases
/// @param seconds Time between each trajectory
/// @param k_order KOMO parameter for the objective function
///////////////////////////////////////////////////////////////////////
void KOMO::SetTiming(const double num_phases, const double num_time_slices, 
                     const double seconds, const double k_order)
{
    for (uint i = 0; i<num_phases; ++i)
    {
        phases.push_back(Phase{i, num_time_slices, std::vector<FeatureSymbol>{FeatureSymbol::FS_none}});
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Add constraints to the phases based on given ID
/// @param phases_ID Vector of phases' ID
/// @param FeatureSymbol Enum for different constraints
///////////////////////////////////////////////////////////////////////
void KOMO::AddConstraint(const std::vector<uint> phases_ID, FeatureSymbol g)
{
    for (auto id : phases_ID)
    {
        if (phases[id].constraints[0] == FeatureSymbol::FS_none ) phases[id].constraints.clear();
        phases[id].constraints.push_back(g);
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Remove all constraints from the phases based on given ID
/// @param phases_ID Vector of phases' ID
///////////////////////////////////////////////////////////////////////
void KOMO::ClearConstraint(const std::vector<uint> phases_ID)
{
    for (auto id : phases_ID)
    {
        phases[id].constraints.clear();
        phases[id].constraints.push_back(FeatureSymbol::FS_none);
    }
}

///////////////////////////////////////////////////////////////////////
/// @brief Delete all phases
/// @param
///////////////////////////////////////////////////////////////////////
void KOMO::Reset()
{
    phases.clear();
}


///////////////////////////////////////////////////////////////////////
/// @brief Find a solution
/// @param
///////////////////////////////////////////////////////////////////////
KomoStatus KOMO::Optimize()
{

}