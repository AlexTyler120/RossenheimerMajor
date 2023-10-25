#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalBased.h"

GoalBased::GoalBased()
{
    ROS_INFO("[CTor]: Based");
}

GoalBased::GoalBased(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("Goal Created");
    status = true;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = coord_x;
    goal.target_pose.pose.position.y = coord_y;
    goal.target_pose.pose.orientation.w = orientation;
    ROS_INFO("Orientation (w): %f", orientation);
}

GoalBased::~GoalBased()
{
    ROS_INFO("[DTor]: So not Based.");
}

void GoalBased::actionTask()
{
    ROS_INFO("Resupplying rossenheimer");
    status = false;
}