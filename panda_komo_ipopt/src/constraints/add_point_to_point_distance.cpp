#include <panda_komo_ipopt/constraints/add_point_to_point_distance.h>

AddPointToPointDistanceConstraint::AddPointToPointDistanceConstraint()
    : AddPointToPointDistanceConstraint("point_to_point_distance_constraint", 20, 0.1)
{
}

AddPointToPointDistanceConstraint::AddPointToPointDistanceConstraint(const std::string& name, const int num_time_slices, const double tolerance)
    : ifopt::ConstraintSet(num_time_slices, name)
{
    _tolerance = tolerance;
}


Eigen::VectorXd AddPointToPointDistanceConstraint::GetValues() const
{
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();

    Eigen::VectorXd object_pos(3);
    object_pos << 0.5, 0, 0.125;

    for (size_t idx=0; idx<20; idx++)
    {
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data()+idx*7, 7);
        auto FK_q = Kinematics::ForwardKinematics(q, true);
        Eigen::VectorXd pos_t(3);
        pos_t << FK_q(0,3), FK_q(1,3), FK_q(2,3);
        auto diff_pos = pos_t - object_pos;
        double l2_norm = sqrt(diff_pos.transpose() * diff_pos);
        g(idx) = l2_norm;
    }
    std::cout << "g(0) = \t" << GetRows() << std::endl;

    return g;
}


AddPointToPointDistanceConstraint::VecBound AddPointToPointDistanceConstraint::GetBounds() const
{
    VecBound bounds(GetRows());
    for (auto &b : bounds)
    {
        b = ifopt::Bounds(_tolerance, ifopt::inf);
    }
    return bounds;
}

void AddPointToPointDistanceConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac_block) const
{
    if (var_set == "x") 
    {
        Vector2d x = GetVariables()->GetComponent("x")->GetValues();

        std::cout << "size of jac_constraint: " << jac_block.size() << std::endl;
    }

}
