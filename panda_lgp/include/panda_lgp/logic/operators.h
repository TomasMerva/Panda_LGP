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

        std::vector<SkeletonEntry> operators;   // TODO: change to private

    private:
  
};


} // namespace