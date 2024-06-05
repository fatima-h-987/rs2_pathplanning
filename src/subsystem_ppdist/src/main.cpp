#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <laserprocessing.h> 
#include "nav_msgs/Odometry.h"

// Global Variables
std::vector<geometry_msgs::PoseStamped> waypoints;
bool path_received = false;
bool goals_received = false;

//A callback for odometry
void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    // robotPose_ = pose; // We copy the pose here
}


// Callback function to process the point messages
void pathCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point waypoint_data;
    waypoint_data = *msg;  // Store the received message in the global variable
    path_received = true;

    std::cout << "Waypoint: {" << waypoint_data.x << " , " << waypoint_data.y << "} received." << std::endl;

    geometry_msgs::PoseStamped waypoint_pose;
    waypoint_pose.header.frame_id = "map"; // Set the frame of reference
    waypoint_pose.pose.position = waypoint_data; // Set the goal position x-coordinate
    waypoint_pose.pose.orientation.w = 1.0; // Orientation should be set properly
    waypoints.push_back(waypoint_pose); // Publish the goal
    //ROS_INFO("Received point message");
}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point goal_point;
    goal_point = *msg;  // Store the received message in the global variable
    goals_received = true;
    //ROS_INFO("Received point message");
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pathplanning");
    ros::NodeHandle nh;

    // 2D NAV Goal uses Dijkstra or A* algorithm. How does it respond to obstables?
    ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Subscriber sub_odom = nh.subscribe("/odom", 100, odomCallback);
    ros::Subscriber sub_waypts = nh.subscribe("/thepath", 5000, pathCallback);
    ros::Subscriber sub_goals = nh.subscribe("/therandomgoals", 100, goalCallback);

    int current_waypoint = 0;

    ros::Rate rate(1.0);

    // Publish each waypoint in sequence
    std::cout << "Starting to pushback points..." << std::endl;
    while (ros::ok() && current_waypoint < waypoints.size())
    {
        goal_publisher.publish(waypoints[current_waypoint]);

        // Sleep briefly between waypoints to control the speed
        ros::Duration(21.5).sleep(); // Adjust the duration as needed
    
        ros::Duration waypoint_duration(5.0);

        ros::Time start_time = ros::Time::now();
        while (ros::ok() && ros::Time::now() - start_time < waypoint_duration) 
        {
            ros::spinOnce();
            rate.sleep();
        }

        current_waypoint++;
    }

    return 0;
}