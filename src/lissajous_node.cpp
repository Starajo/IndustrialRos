#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <vector>
#include <cmath> 
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lissajous_curve_robot");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    std::string base_frame = group.getPlanningFrame();
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.loadMarkerPub("/rviz_visual_tools");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>(
        "/move_group/display_planned_path", 1, true);

    ros::Publisher ideal_traj_pub = nh.advertise<geometry_msgs::PoseArray>(
        "/ideal_lissajous_path", 1, true);

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());

    // Lissajous parameters
    double A = 0.05; // Amplitude in X (m)
    double B = 0.05; // Amplitude in Y (m)
    double a = 3.0; // Frequency en X
    double b = 4.0; // Frequency Y
    double delta = M_PI / 2.0; 
    int num_points = 200;

    // End effector initial pose
    geometry_msgs::Pose start_pose = group.getCurrentPose().pose;

    // Generate the waypoints of the ideal trajectory
    std::vector<geometry_msgs::Pose> waypoints;
    ROS_INFO("End effector link: %s",group.getEndEffectorLink().c_str());
    //waypoints.push_back(start_pose);

    geometry_msgs::PoseArray ideal_traj_msg;
    ideal_traj_msg.header.stamp = ros::Time::now();
    ideal_traj_msg.header.frame_id = group.getPlanningFrame(); 
    
    int cycles = 1;
    for (int i = 0; i <= num_points; ++i)
    {
        double t = static_cast<double>(i) / num_points * 2 * M_PI * cycles;
        geometry_msgs::Pose target_pose = start_pose;

        target_pose.position.x = start_pose.position.x + A * sin(a * t + delta);
        target_pose.position.y = start_pose.position.y + B * sin(b * t);
        target_pose.position.z = 0.4;


        waypoints.push_back(target_pose);
        ideal_traj_msg.poses.push_back(target_pose);
    }

    // Publish the ideal trajectory 
    ideal_traj_pub.publish(ideal_traj_msg);
    ROS_INFO("Published %zu waypoints for the ideal Lissajous curve.", ideal_traj_msg.poses.size());
    visual_tools.publishPath(waypoints, rviz_visual_tools::ORANGE, rviz_visual_tools::SMALL);
    visual_tools.trigger();

    std::ofstream output_file("trayectoria_ideal.csv");
    if (output_file.is_open()) {
        output_file << "x,y,z\n";
        for (const auto& pose : waypoints) {
            output_file << pose.position.x << ","
                        << pose.position.y << ","
                        << pose.position.z << "\n";
        }
        output_file.close();
        ROS_INFO("Ideal trajectory waypoints saved to trayectoria_ideal.csv");
    } else {
        ROS_ERROR("Unable to open file to save ideal trajectory waypoints.");
    }
    // Cartesian Trajectory
    moveit_msgs::RobotTrajectory trajectory;
    double eef_step = 0.001; 
    double jump_threshold = 0.5; // avoid joint discontinuity
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);


    ROS_INFO("Cartesian path (%.2f%% achieved)", fraction * 100.0);

    // Execute the trajectory if it is complete
    if (fraction == 1.0)
    {
        ROS_INFO("Planning successful, executing the Lissajous trajectory.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        group.execute(plan);
    }
    else
    {
        ROS_WARN("Only %.2f%% of the Cartesian path was planned successfully.", fraction * 100.0);
    }
    
    //ros::Publisher ee_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose", 10);
    //ros::Duration(1.0).sleep();

    //double trajectory_duration = 0.0;
    //for (const auto& point : trajectory.joint_trajectory.points)
    //{
    //    double t = point.time_from_start.toSec();
    //    if (t > trajectory_duration)
    //        trajectory_duration = t;
    //}

    //ros::Duration duration(trajectory_duration);

    //ros::Rate rate(50);  // 50 Hz
    //ros::Time start_time = ros::Time::now();

    //while (ros::Time::now() - start_time < duration && ros::ok())
    //{
    //    geometry_msgs::PoseStamped ee_pose = group.getCurrentPose();
    //    ee_pose.header.stamp = ros::Time::now();
    //    ee_pose.header.frame_id = group.getPlanningFrame();
    //    ee_pose_pub.publish(ee_pose);
    //    rate.sleep();
    //}
    


    ros::shutdown();
    return 0;
}
