#include <ros/ros.h>
#include <panda_lgp/utils/kinematics.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nlopt");
    ros::NodeHandle nh;

    Eigen::VectorXd q(7);
    q << -2.407308, 1.555370, -2.102171, -0.011156, 1.100545, 3.230793, -2.651568;

    auto J = kinematics::GeometricJacobian(q, false);
    std::cout << J << std::endl;

    return 0;
}