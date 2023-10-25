#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalFlood.h"

GoalFlood::GoalFlood(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("Goal Created");
    status = true;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = coord_x;
    goal.target_pose.pose.position.y = coord_y;
    goal.target_pose.pose.orientation.w = orientation;
}

GoalFlood::~GoalFlood()
{
    ROS_INFO("[DTor]: Flood blocked");
} 


void GoalFlood::actionTask()
{
    ROS_INFO("blocking Flood");
    status = false;
}