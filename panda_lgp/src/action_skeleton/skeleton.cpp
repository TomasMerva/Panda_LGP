#include <panda_lgp/action_skeleton/skeleton.h>

namespace logic
{

Skeleton::Skeleton()
{
    // This is only range in y coordinate
    _grey_region = {-0.5, 0.2};
    _red_region = {0.2, 0.5};

    // The cube position [x,y,z]
    // _cube_position = {0.4, -0.3, 0.025};
    _cube_frame << 1, 0, 0, 0.4,
                   0, 1, 0, -0.3,
                   0, 0, 1, 0.025,
                   0, 0, 0, 1;
    _cube_pos = std::vector<double>(_cube_frame.col(3).data(), _cube_frame.col(3).data()+3);
}



Skeleton::Skeleton(std::vector<SkeletonEntry> op)
{
    operators = op;
}

void Skeleton::SetKOMO(KOMO *komo)
{
    uint num_time_slices = 20;
    // x_dim = q(7) + cube[x,y,z]
    int x_dim = 10;
    std::vector<Constraint::ConstraintData> constraints_data(operators.size());

    for (uint i=0; i<operators.size(); ++i)
    {
        KOMO::Phase phase_i;
        phase_i.lower_bounds = std::vector<double>(x_dim);
        phase_i.upper_bounds = std::vector<double>(x_dim);
        // phase_i.x_init = std::vector<double>(x_dim);    // TODO:
        switch (operators[i].action_name)
        {
            case SkeletonAction::MoveF:
                // General info
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveF";
                phase_i.num_time_slices = num_time_slices;
                // ---- Constraints ----
                // 1. Be in _grey_region:
                phase_i.constraints.push_back(Constraint::AxisInRegion);
                constraints_data[i].idx = i;
                constraints_data[i].region = _grey_region;
                phase_i.constraints_data.push_back(constraints_data[i]);
                // ---- Boundaries ----
                // 1. Joint Limits
                komo->AddJointLimits(phase_i.lower_bounds, phase_i.upper_bounds);
                // Init guess
                phase_i.x_init.insert(komo->configuration.q_act.begin(), _cube_pos.begin(), _cube_pos.end());
                komo->phases.push_back(phase_i);
                break;
            case SkeletonAction::MoveH:
                // General info
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveH";
                phase_i.num_time_slices = num_time_slices;
                // ---- Constraints ----
                // 1. Be in _red_region:
                phase_i.constraints.push_back(Constraint::AxisInRegion);
                constraints_data[i].idx = i;
                constraints_data[i].region = _red_region;
                phase_i.constraints_data.push_back(constraints_data[i]);
                // ---- Boundaries ----
                // 1. Joint Limits
                komo->AddJointLimits(phase_i.lower_bounds, phase_i.upper_bounds);
                // ---- Init guess ----
                phase_i.x_init.insert(komo->configuration.q_act.begin(), _cube_pos.begin(), _cube_pos.end());
                komo->phases.push_back(phase_i);
                break;

            // case SkeletonAction::Pick:
            //     std::cout << "Pick\n";
            //     phase_i.ID = i;
            //     phase_i.symbolic_name = "Pick";
            //     phase_i.num_time_slices = num_time_slices;
                
            //     komo->phases.push_back(phase_i);
            //     break;
            // case SkeletonAction::Place:
            //     std::cout << "Place\n";
            //     phase_i.ID = i;
            //     phase_i.symbolic_name = "Place";
            //     phase_i.num_time_slices = num_time_slices;
            //     komo->phases.push_back(phase_i);
            //     break;
            default:
                std::cout << "No action\n";
                break;
        } // switch
    } // for -> scanning through action skeleton


  
} // Transform it into NLP model




} //namespace