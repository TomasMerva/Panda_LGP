#pragma once

#include <vector>
#include <string>
#include <panda_lgp/action_skeleton/operators.h>
#include <panda_lgp/KOMO/komo.h>
#include <panda_lgp/constraints/constraints.h>

namespace logic
{

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
        void SetKOMO(KOMO *komo);

        std::vector<SkeletonEntry> operators;   // TODO: change to private

    private:
        // TODO: it is not general!!!
        std::vector<double> _grey_region = std::vector<double>(2);
        std::vector<double> _red_region = std::vector<double>(2);
        std::vector<double> _cube_position = std::vector<double>(3);
        // TODO: maybe add actual pose of the robot? or maybe read it from KOMO
  
};

} // namespace