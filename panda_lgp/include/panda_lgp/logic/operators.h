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
    std::vector<FeatureSymbol> g;
    MoveF_Action(std::string frame, std::vector<double> region_init, std::vector<double> region_goal, std::vector<FeatureSymbol> g) 
        : robot_frame(frame), region_init(region_init), region_goal(region_goal), g(g) {};
};

struct MoveH_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> region_init = std::vector<double>(3);
    std::vector<double> region_goal = std::vector<double>(3);
    std::vector<FeatureSymbol> g;
    MoveH_Action(std::string robot_frame, std::string object_frame, std::vector<double> region_init, std::vector<double> region_goal, std::vector<FeatureSymbol> g) 
        : robot_frame(robot_frame), object_frame(object_frame), region_init(region_init), region_goal(region_goal), g(g) {};
};


struct Pick_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> position_object = std::vector<double>(3);
    std::vector<double> grasp = std::vector<double>(3);
    FeatureSymbol mode_switch;
    std::vector<FeatureSymbol> g;
    Pick_Action(std::string robot_frame, std::string object_frame, std::vector<double> position_object, std::vector<double> grasp, FeatureSymbol mode_switch, std::vector<FeatureSymbol> g)
        : robot_frame(robot_frame), object_frame(object_frame), position_object(position_object), grasp(grasp), mode_switch(mode_switch), g(g) {};
};


struct Place_Action
{
    std::string robot_frame;
    std::string object_frame;
    std::vector<double> place_region = std::vector<double>(6);
    FeatureSymbol mode_switch;
    std::vector<FeatureSymbol> g;
    Place_Action(std::string robot_frame, std::string object_frame, std::vector<double> place_region, FeatureSymbol mode_switch, std::vector<FeatureSymbol> g)
        : robot_frame(robot_frame), object_frame(object_frame), place_region(place_region), mode_switch(mode_switch), g(g) {};
};



struct SkeletonEntry
{
    SkeletonAction action_name;
    std::vector<std::string> frames;
    SkeletonEntry(SkeletonAction action_name, std::vector<std::string> frames) : action_name(action_name), frames(frames) {};
};

class Skeleton
{
    public:
        Skeleton();
        Skeleton(std::vector<SkeletonEntry> op);
        
        // TODO: create a method that transforms the "operators" vector into constraints with continuous values


        std::vector<SkeletonEntry> operators;   // TODO: change to private

    private:
  
};


} // namespace