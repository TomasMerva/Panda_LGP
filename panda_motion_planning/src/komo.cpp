#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <panda_gazebo_controllers/JointPosition.h>

#include <panda_motion_planning/komo_lib.h>
#include <gnuplot_module/gnuplot_module.h>

#include <panda_motion_planning/panda_kinematics.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>


std::vector<double> q_start(7);

void JointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    q_start = msg->position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "komo");
    ros::NodeHandle nh;

    // Arguments from launch file
    std::string execute_arg, visualize_arg, plot_arg;
    execute_arg = argv[1];
    visualize_arg = argv[2];
    plot_arg = argv[3];

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher joint_pub = nh.advertise<panda_gazebo_controllers::JointPosition>("/panda/joint_position_goal", 1000);
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);

    const int num_joints = 7;
    const int num_time_slices = 20;
    // std::vector<double> q_goal{-0.2919, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397};
    std::vector<double> q_goal{-0.2919, -0.785398163, 0, -1.5, 0, 1.57079632679, 0.785398163397};

    // safety break
    ros::Duration(1).sleep();

    // KOMO
    KOMO komo(num_joints, num_time_slices, 2);
    komo.UpdateStates(q_start, q_goal);
    // each vector in results is separate joint, each joint consists of num_time_slices elements
    std::vector<std::vector<double>> results = komo.Optimize();





    // Plot results
    if (plot_arg == "true")
    {
        GnuplotModule plot;
        // plot.Subplot_Yranges(komo.GetJointLimits());
        std::vector<double> time = linspace(0, 5, num_time_slices);
        std::vector<std::string> labels{"q1", "q2", "q3", "q4", "q5", "q6", "q7"};
        plot.SubplotData(time, results, labels);
    }

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    while(true)
    {
        if (visualize_arg == "true")
        {

            std::unique_ptr<panda_kinematics::Kinematics> kin = std::make_unique<panda_kinematics::Kinematics>(&nh);

            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "world";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "trajectory";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = 0;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.01;
            line_strip.color.g = 1.0;
            line_strip.color.a = 1.0;
            
            for (size_t t=0; t < num_time_slices; t++)
            {
                std::array<double, 7> timestep_joints{results[0][t],
                                                    results[1][t],
                                                    results[2][t],
                                                    results[3][t],
                                                    results[4][t],
                                                    results[5][t],
                                                    results[6][t] };
                Eigen::Matrix4d T = kin->ForwardKinematics(timestep_joints);
                
                geometry_msgs::Point p;
                p.x = T(0,3);
                p.y = T(1,3);
                p.z = T(2,3);
                line_strip.points.push_back(p);

            }
            marker_pub.publish(line_strip);
        }

        std::string answer;
        std::cout << "Do you want to execute?";
        std::cin >> answer;

        // Execute trajectory
        if (answer == "y")
        {
            double dt = 5.0 / static_cast<double>(num_time_slices);
            for (int i=0; i<num_time_slices; i++)
            {
                panda_gazebo_controllers::JointPosition msg;
                msg.joint_position[0] = results[0][i];
                msg.joint_position[1] = results[1][i];
                msg.joint_position[2] = results[2][i];
                msg.joint_position[3] = results[3][i];
                msg.joint_position[4] = results[4][i];
                msg.joint_position[5] = results[5][i];
                msg.joint_position[6] = results[6][i];
                
                joint_pub.publish(msg);     
                ros::Duration(dt).sleep();
            }
        }
        ros::Duration(0.1).sleep();
    }
    
    ros::waitForShutdown();
    return 0;
}