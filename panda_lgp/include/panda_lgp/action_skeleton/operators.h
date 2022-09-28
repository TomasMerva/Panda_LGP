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
    std::vector<FeatureSymbol> g = std::vector<FeatureSymbol>{FeatureSymbol::FS_AxisInRange};
};

struct MoveH_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> region_init = std::vector<double>(3);
    std::vector<double> region_goal = std::vector<double>(3);
    std::vector<FeatureSymbol> g = std::vector<FeatureSymbol>{FeatureSymbol::FS_ObjectInRegion};
};


struct Pick_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> position_object = std::vector<double>(3);
    std::vector<double> grasp = std::vector<double>(3);
    FeatureSymbol mode_switch = FeatureSymbol::FS_PickedObject;
    std::vector<FeatureSymbol> g = std::vector<FeatureSymbol>{FeatureSymbol::FS_Grasp};
};


struct Place_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> place_region = std::vector<double>(6);
    FeatureSymbol mode_switch;
    std::vector<FeatureSymbol> g = std::vector<FeatureSymbol>{FeatureSymbol::FS_AxisInRegion, FeatureSymbol::FS_AxisInRegion};
};






} // namespace