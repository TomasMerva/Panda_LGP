#pragma once

#include <vector>
#include <string>
#include <panda_lgp/constraints/constraints.h>

namespace logic
{

enum SkeletonAction
{
    MoveF,
    Pick,
    MoveH,
    Place
};


struct MoveF_Action
{
    std::string robot_frame;
    std::vector<double> region_init = std::vector<double>(3);
    std::vector<double> region_goal = std::vector<double>(3);
    // std::vector<ConstraintSymbol> g = std::vector<ConstraintSymbol>{ConstraintSymbol::FS_AxisInRange};
};

struct MoveH_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> region_init = std::vector<double>(3);
    std::vector<double> region_goal = std::vector<double>(3);
    // std::vector<ConstraintSymbol> g = std::vector<ConstraintSymbol>{ConstraintSymbol::FS_ObjectInRegion};
};


struct Pick_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> position_object = std::vector<double>(3);
    std::vector<double> grasp = std::vector<double>(3);
    // ConstraintSymbol mode_switch = ConstraintSymbol::FS_PickedObject;
    // std::vector<ConstraintSymbol> g = std::vector<ConstraintSymbol>{ConstraintSymbol::FS_Grasp};
};


struct Place_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> place_region = std::vector<double>(6);
    // ConstraintSymbol mode_switch;
    // std::vector<ConstraintSymbol> g = std::vector<ConstraintSymbol>{ConstraintSymbol::FS_AxisInRegion, ConstraintSymbol::FS_AxisInRegion};
};






} // namespace