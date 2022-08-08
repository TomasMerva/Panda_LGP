#include <panda_motion_planning/constraints/add_fixed_orientation_axis.h>


double Constraint::AddFixedOrientationAxis(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    int num_variables = 7;
    Eigen::VectorXd fixed_rot(3);

    AddFixedOrientationAxisData *d = reinterpret_cast<AddFixedOrientationAxisData*>(data);
    auto idx = d->idx;
    auto axis_idx = d->axis_idx; // 0->x, 1->y, 2->z
    fixed_rot << d->axis_vector_1, d->axis_vector_2, d->axis_vector_3;

    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*num_variables, num_variables);
    auto FK_q = Kinematics::ForwardKinematics(q, true);    
    Eigen::VectorXd diff_rot = FK_q.block(0, axis_idx, 3, 1) - fixed_rot;
    double squared_error = diff_rot.transpose() * diff_rot;

    if (!grad.empty())
    {
        double delta_q = 0.001;
        std::fill(grad.begin(), grad.end(), 0);

        // Compute numerical Jacobian
        Eigen::MatrixXd J(3, num_variables);
        for (int i=0; i<num_variables; i++)
        {
            auto temp = q;
            temp(i) += delta_q; // ad delta_q to the q(i)
            auto FK_with_deltaq = Kinematics::ForwardKinematics(temp, true); //compute FK with q(i)+delta_q
            J(0, i) = (FK_with_deltaq(0,3) - FK_q(0,3)) / delta_q;  // dx / dq(i)
            J(1, i) = (FK_with_deltaq(1,3) - FK_q(1,3)) / delta_q;  // dy / dq(i)
            J(2, i) = (FK_with_deltaq(2,3) - FK_q(2,3)) / delta_q;  // dz / dq(i)
        }
        // Compute the derivative of a constraint function
        auto dg = 2*diff_rot.transpose()*J;
        // Copy the dg(x) to std::vector<double> grad
        for (int i=0; i<num_variables; i++)
        {
            grad[i + idx*num_variables] = dg(i);
        }
    }
    return squared_error;
}

