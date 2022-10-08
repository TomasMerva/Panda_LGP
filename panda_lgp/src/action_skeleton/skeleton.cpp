#include <panda_lgp/action_skeleton/skeleton.h>

namespace logic
{

Skeleton::Skeleton()
    : _x_phase_dim(7+6+3) // q(7) + 6DoF(man_frame) + [x,y,z] cube = 16
{
    // This is only range in y coordinate
    _grey_region = {-0.5, 0.2};
    _red_region = {0.2, 0.5};

    // The cube position [x,y,z]
    _cube_frame << 1, 0, 0, 0.4,
                   0, 1, 0, -0.3,
                   0, 0, 1, 0.0345,
                   0, 0, 0, 1;
    _cube_pos = std::vector<double>(_cube_frame.col(3).data(), _cube_frame.col(3).data()+3);
}



Skeleton::Skeleton(std::vector<SkeletonEntry> op)
    : _x_phase_dim(7+6+3) // q(7) + 6DoF(man_frame) + [x,y,z] cube = 16
{
    operators = op;

    // This is only range in y coordinate
    _grey_region = {-0.5, 0.2};
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
    phase_x_init.insert(phase_x_init.end(), _cube_pos.begin(), _cube_pos.end());
}



void 
Skeleton::SetKOMO(KOMO *komo)
{
    std::vector<Constraint::ConstraintData> constraints_data(operators.size());

    for (uint i=0; i<operators.size(); ++i)
    {
        KOMO::Phase phase_i;
        phase_i.lower_bounds = std::vector<double>(_x_phase_dim);
        phase_i.upper_bounds = std::vector<double>(_x_phase_dim);
        switch (operators[i].action_name)
        {
            case SkeletonAction::MoveF:
                // ---- General info ----
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveF";
                //  1. Be in _grey_region:
                phase_i.constraints.push_back(Constraint::AxisInRegion);
                constraints_data[i].idx = i;
                constraints_data[i].num_phase_variables = _x_phase_dim;
                constraints_data[i].region = _grey_region;
                phase_i.constraints_data.push_back(constraints_data[i]);
                // ---- Boundaries ----
                SetBoundariesForPhase(komo, phase_i.lower_bounds, phase_i.upper_bounds);
                // ---- Init guess ----
                SetInitGuessForPhase(komo, phase_i.x_init);
                // Add phase to the vector
                komo->phases.push_back(phase_i);
                break;

            case SkeletonAction::MoveH:
                // ---- General info ----
                phase_i.ID = i;
                phase_i.symbolic_name = "MoveH";
                // ---- Constraints ----
                //  1. Be in _red_region:
                phase_i.constraints.push_back(Constraint::AxisInRegion);
                constraints_data[i].idx = i;
                constraints_data[i].num_phase_variables = _x_phase_dim;
                constraints_data[i].region = _red_region;
                phase_i.constraints_data.push_back(constraints_data[i]);
                // ---- Boundaries ----
                SetBoundariesForPhase(komo, phase_i.lower_bounds, phase_i.upper_bounds);
                // ---- Init guess ----
                SetInitGuessForPhase(komo, phase_i.x_init);
                // Add phase to the vector
                komo->phases.push_back(phase_i);
                break;

            default:
                std::cout << "No action is in the skeleton\n";
                break;
        } // switch
    } // for -> scanning through action skeleton 
}




} //namespace