#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <iostream>
#include <vector>
#include <cmath> // For sin and cos functions

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lissajous_curve_robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0); // Give some time for ROS to initialize

    moveit::planning_interface::MoveGroupInterface group("panda_arm");

    // Optional: PlanningSceneInterface if you want to add collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Publishers for displaying the trajectory in RViz
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());

    // --- Lissajous Curve Parameters ---
    double A = 0.1; // Amplitude in X direction (meters)
    double B = 0.1; // Amplitude in Y direction (meters)
    double a = 3.0; // Frequency parameter for X
    double b = 2.0; // Frequency parameter for Y
    double delta = M_PI / 2.0; // Phase shift (e.g., M_PI/2 for a circle-like shape)
    double num_points = 500; // Number of points to generate for the curve
    double cycle_duration = 10.0; // Total time to complete one cycle of the curve (seconds)

    // --- Define the start pose of the end-effector ---
    // This is crucial. The Cartesian path planning starts from the current pose of the end-effector.
    // If your robot is not at a known, safe starting position, it's good practice to move it there first.
    geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
    // You might want to adjust the initial Z position to be above any potential obstacles
    // For example: start_pose.position.z += 0.1;

    // --- Generate Waypoints for the Lissajous Curve ---
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose); // Always start with the current pose

    for (int i = 0; i <= num_points; ++i)
    {
        double t = (double)i / num_points; // Normalized time from 0 to 1

        geometry_msgs::Pose target_pose = start_pose; // Start with the initial orientation and base position

        // Calculate X and Y coordinates based on Lissajous formulas
        target_pose.position.x = start_pose.position.x + A * sin(a * t * 2 * M_PI);
        target_pose.position.y = start_pose.position.y + B * sin(b * t * 2 * M_PI + delta);
        // Keep Z constant for a 2D curve in the XY plane.
        // If you want a 3D Lissajous curve, you'd add a Z component as well.
        // target_pose.position.z = start_pose.position.z;

        // Keep the orientation constant (as defined in start_pose)
        // If you need the end-effector to reorient, you'd calculate that here.

        waypoints.push_back(target_pose);
    }

    ROS_INFO("Generated %zu waypoints for the Lissajous curve.", waypoints.size());

    // --- Cartesian Path Planning ---
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step: The maximum distance (in meters) between consecutive points in the Cartesian path. Smaller values result in smoother paths but more points.
                                                 0.0,   // jump_threshold: The maximum distance (in meters) the end-effector can "jump" between points in Cartesian space before it's considered a collision or invalid path. Set to 0.0 for strict Cartesian paths.
                                                 trajectory,
                                                 true); // avoid_collisions: Whether to check for collisions during path computation. Set to true for safe operation.

    ROS_INFO("Cartesian path (%.2f%% achieved)", fraction * 100.0);

    // --- Execute the Trajectory ---
    if (fraction == 1.0)
    {
        ROS_INFO("Planning successful, moving the robot to draw the Lissajous curve.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;
        group.execute(plan); // Execute the planned trajectory
    }
    else
    {
        ROS_WARN("Failed to plan full Cartesian path for Lissajous curve. Only %.2f%% achieved.", fraction * 100.0);
    }

    ros::shutdown();
    return 0;
}