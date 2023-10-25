#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalFire.h"

GoalFire::GoalFire(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("Goal Created");
    status = true;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = coord_x;
    goal.target_pose.pose.position.y = coord_y;
    goal.target_pose.pose.orientation.w = orientation;
}

GoalFire::~GoalFire()
{
    ROS_INFO("[DTor]:Fire Extinguished");
} 


void GoalFire::actionTask()
{
    ROS_INFO("Extinguishing Fire");
    status = false;
}