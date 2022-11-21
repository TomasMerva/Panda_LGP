#pragma once
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <Eigen/Dense>

class VisualizeRviz
{
    public:
        VisualizeRviz(ros::NodeHandle &nh);
        void VisualizeTrajectory(Eigen::ArrayXXd x);
        void VisualizeObstacle(std::vector<double> position);

    private:
        ros::Publisher _marker_pub;
};

