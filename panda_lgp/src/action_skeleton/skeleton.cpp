#include <panda_lgp/action_skeleton/skeleton.h>

namespace logic
{

Skeleton::Skeleton()
{
    // This is only range in y coordinate
    _grey_region = {-0.5, 0.2};
    _red_region = {0.2, 0.5};

    // The cube position [x,y,z]
    _cube_position = {0.4, -0.3, 0.025};
}


Skeleton::Skeleton(std::vector<SkeletonEntry> op)
{
    operators = op;
}

void Skeleton::SetKOMO(KOMO *komo)
{
    uint num_time_slices = 20;

    for (uint i=0; i<operators.size(); ++i)
    {
        KOMO::Phase phase_i;
        switch (operators[i].action_name)
        {
            case SkeletonAction::MoveF:
                std::cout << "MoveF\n";
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveF";
                phase_i.num_time_slices = num_time_slices;
                break;
            case SkeletonAction::MoveH:
                std::cout << "MoveH\n";
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveH";
                phase_i.num_time_slices = num_time_slices;
                break;
            case SkeletonAction::Pick:
                std::cout << "Pick\n";
                phase_i.ID = i;
                phase_i.symbolic_name = "Pick";
                phase_i.num_time_slices = num_time_slices;
                break;
            case SkeletonAction::Place:
                std::cout << "Place\n";
                phase_i.ID = i;
                phase_i.symbolic_name = "Place";
                phase_i.num_time_slices = num_time_slices;
                break;
            default:
                break;
        }
    }
}


} //namespace