#pragma once

#include <Eigen/Dense>
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
        void SetBoundariesForPhase(KOMO *komo, std::vector<double> &lb, std::vector<double> &ub);
        void SetInitGuessForPhase(KOMO *komo, std::vector<double> &phase_x_init);

        // TODO: it is not general!!!
        std::vector<double> _grey_region = std::vector<double>(2);
        std::vector<double> _red_region = std::vector<double>(2);
        Eigen::Matrix4d _cube_frame;
        std::vector<double> _cube_pos;  //TODO: delete
        uint _x_phase_dim;
  
};

} // namespace