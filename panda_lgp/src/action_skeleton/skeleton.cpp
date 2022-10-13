#include <panda_lgp/action_skeleton/skeleton.h>

namespace logic
{

Skeleton::Skeleton()
    : _x_phase_dim(7+6) // q(7) + 6DoF(man_frame)
{
    // This is only range in y coordinate
    _grey_region = {-0.5, 0.2};
    _red_region = {0.15, 0.5};

    // The cube position [x,y,z]
    _cube_frame << 1, 0, 0, 0.4,
                   0, 1, 0, -0.3,
                   0, 0, 1, 0.0345,
                   0, 0, 0, 1;
    _cube_pos = std::vector<double>(_cube_frame.col(3).data(), _cube_frame.col(3).data()+3);
}



Skeleton::Skeleton(std::vector<SkeletonEntry> op)
    : _x_phase_dim(7+6) // q(7) + 6DoF(man_frame)
{
    operators = op;

    // This is only range in y coordinate
    _grey_region = {-0.5, 0.15};
    _red_region = {0.2, 0.5};

    // The cube position [x,y,z]
    _cube_frame << 1, 0, 0, 0.4,
                   0, 1, 0, -0.3,
                   0, 0, 1, 0.0345,
                   0, 0, 0, 1;
    _cube_pos = std::vector<double>(_cube_frame.col(3).data(), _cube_frame.col(3).data()+3);
}



void 
Skeleton::SetBoundariesForPhase(KOMO *komo, std::vector<double> &lb, std::vector<double> &ub)
{
    komo->AddJointLimits(lb, ub);
    // Have to add boundaries for other decision variables so they are not 0
    for (uint idx_bound=7; idx_bound<_x_phase_dim; ++idx_bound)
    {
        lb[idx_bound] = -HUGE_VAL;         // inf number
        ub[idx_bound] = HUGE_VAL;          // inf number
    }
}

void
Skeleton::SetInitGuessForPhase(KOMO *komo, std::vector<double> &phase_x_init)
{
    // Use act q(7) as init
    phase_x_init.insert(phase_x_init.end(), komo->configuration.q_act.begin(), komo->configuration.q_act.end());
    // Add zeros for manipulation_frame
    std::vector<double> manipulation_frame(6, 0);
    phase_x_init.insert(phase_x_init.end(), manipulation_frame.begin(), manipulation_frame.end());
    // Add object coordinates as init
    // phase_x_init.insert(phase_x_init.end(), _cube_pos.begin(), _cube_pos.end());
}



void 
Skeleton::SetKOMO(KOMO *komo)
{
    // std::vector<Constraint::ConstraintData> constraints_data(operators.size());

    // Add init phase
    KOMO::Phase phase_init;
    phase_init.ID = 0;
    phase_init.symbolic_name = "Init_state";
    phase_init.lower_bounds = std::vector<double>(_x_phase_dim);
    phase_init.upper_bounds = std::vector<double>(_x_phase_dim);
    komo->AddJointLimits(phase_init.lower_bounds, phase_init.upper_bounds);
    phase_init.x_init.insert(phase_init.x_init.end(), komo->configuration.q_act.begin(), komo->configuration.q_act.end());
    phase_init.x.insert(phase_init.x.end(), komo->configuration.q_act.begin(), komo->configuration.q_act.end());
    komo->phases.push_back(phase_init);

    // Add phases from skeleton
    for (uint i=1; i<=operators.size(); ++i)    // constraints will overflow because of i<=operators.size
    {
        KOMO::Phase phase_i;
        phase_i.lower_bounds = std::vector<double>(_x_phase_dim);
        phase_i.upper_bounds = std::vector<double>(_x_phase_dim);
        switch (operators[i-1].action_name)
        {
            case SkeletonAction::MoveF:
            {
                // ---- General info ----
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveF";
                // ---- Constraints ----
                //  1. Be in region
                // phase_i.constraints.push_back(Constraint::AxisInRegion);
                Constraint::ConstraintData data_AxisInRegion;
                data_AxisInRegion.idx = i-1;
                data_AxisInRegion.num_phase_variables = _x_phase_dim;
                if (operators[i-1].frames[2] == "grey_region")
                {
                    data_AxisInRegion.region = _grey_region; // target region
                }
                else if (operators[i-1].frames[2] == "red_region")
                {
                    data_AxisInRegion.region = _red_region; // target region
                }                
                phase_i.constraints_data.push_back(data_AxisInRegion);
                // ---- Boundaries ----
                SetBoundariesForPhase(komo, phase_i.lower_bounds, phase_i.upper_bounds);
                // ---- Init guess ----
                SetInitGuessForPhase(komo, phase_i.x_init);
                // Add phase to the vector
                komo->phases.push_back(phase_i);
                break;
            }
            case SkeletonAction::MoveH:
            {
                // ---- General info ----
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveH";
                // ---- Constraints ----
                //  1. Be in region
                // phase_i.constraints.push_back(Constraint::AxisInRegion);
                Constraint::ConstraintData data_AxisInRegion;
                data_AxisInRegion.idx = i-1;
                data_AxisInRegion.num_phase_variables = _x_phase_dim;
                if (operators[i-1].frames[3] == "grey_region")
                {
                    data_AxisInRegion.region = _grey_region; // target region
                }
                else if (operators[i-1].frames[3] == "red_region")
                {
                    data_AxisInRegion.region = _red_region; // target region
                }                
                phase_i.constraints_data.push_back(data_AxisInRegion); 
                // ---- Boundaries ----
                SetBoundariesForPhase(komo, phase_i.lower_bounds, phase_i.upper_bounds);
                // ---- Init guess ----
                SetInitGuessForPhase(komo, phase_i.x_init);
                // Add phase to the vector
                komo->phases.push_back(phase_i);
                break;
            }
            default:
                std::cout << "No action is in the skeleton\n";
                break;
        } // switch
    } // for -> scanning through action skeleton 
}




} //namespace