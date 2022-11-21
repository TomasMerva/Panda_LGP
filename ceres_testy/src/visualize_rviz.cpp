#include "ceres_testy/visualize_rviz.h"

VisualizeRviz::VisualizeRviz(ros::NodeHandle &nh)
{
    _marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

void VisualizeRviz::VisualizeTrajectory(Eigen::ArrayXXd x)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::POINTS;
    line_strip.scale.x = 0.1;
    line_strip.scale.y = 0.1;
    line_strip.scale.z = 0.1;


    std_msgs::ColorRGBA marker_color;
    marker_color.g = 1;
    marker_color.a = 1;
    // line_strip.color.g = 1;
    // line_strip.color.a = 1;

    for (size_t t=0; t < x.cols(); t++)
    {
        geometry_msgs::Point p;
        p.x = x.col(t)[0];
        p.y = x.col(t)[1];
        p.z = x.col(t)[2];
        line_strip.points.push_back(p);
        line_strip.colors.push_back(marker_color);
    }
    _marker_pub.publish(line_strip);
    ROS_INFO("Visualizing trajectory...");
}

void VisualizeRviz::VisualizeObstacle(std::vector<double> position)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.4;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    _marker_pub.publish(marker);
    ROS_INFO("Visualizing obstacle...");
}
