#include <panda_motion_planning/constraints/add_point_to_point_distance.h>

double Constraint::AddPointToPointDistanceConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    AddPointToPointDistanceData *d = reinterpret_cast<AddPointToPointDistanceData*>(data);
    int num_variables = 7;
    size_t idx = d->idx; 
    Eigen::VectorXd obj_pos(3);
    obj_pos << d->obj_pos_x, d->obj_pos_y, d->obj_pos_z;
    double tolerance = d->tolerance;
    double delta_q = 0.001;

    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*num_variables, num_variables);
    auto FK_q = Kinematics::ForwardKinematics(q, true);
    Eigen::VectorXd pos_t(3);
    pos_t << FK_q(0,3), FK_q(1,3), FK_q(2,3);
    auto diff_pos = obj_pos - pos_t;
    double l2_norm = - sqrt((diff_pos.transpose()*diff_pos)[0]) + 0.3;

    if (!grad.empty())
    {
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
        auto dg = diff_pos.transpose()*J;
        // Copy the dg(x) to std::vector<double> grad
        for (int i=0; i<num_variables; i++)
        {
            grad[i + idx*num_variables] = dg(i);
        }
    }
    return l2_norm;
}
