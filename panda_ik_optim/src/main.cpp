#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_ik_optim");
    ros::NodeHandle nh;
    ros::waitForShutdown();
    return 0;
}