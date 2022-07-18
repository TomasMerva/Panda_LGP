#include <panda_motion_planning/constraints/add_point_to_point_distance.h>

AddPointToPointDistanceConstraint::AddPointToPointDistanceConstraint(/* args */)
{
}

AddPointToPointDistanceConstraint::~AddPointToPointDistanceConstraint()
{
}


double AddPointToPointDistanceConstraint::GetValues(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    std::array<double, 7> temp;
    std::copy(x.begin(), x.begin()+7, temp.begin());
    auto T = ForwardKinematics(temp, true);
    
    Eigen::VectorXd position(3);
    position << T(0,3), T(1,3), T(2,3);

    Eigen::VectorXd obj_pos(3);
    obj_pos << 0.5, 0.0, 0.25; 

    double pow_2 = (position-obj_pos).transpose() * (position-obj_pos);
    return std::sqrt(pow_2);
}

void AddPointToPointDistanceConstraint::FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac)
{
    int idx = 0;

}

        // for (auto i : temp)
        // {
        //     std::cout << i << " ";
        // }
        // std::cout << std::endl;