#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalBased.h"

GoalBased::GoalBased(double coord_x, double coord_y, double orientation, int type, int id)
{
    ROS_INFO("[CTor]: Based");
    ROS_INFO("OH SHIT GOAL CREATED OH NO.");
    ROS_INFO("Goal Created");
    status = true;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = coord_x;
    goal.target_pose.pose.position.y = coord_y;
    goal.target_pose.pose.orientation.w = orientation;    

    Target_x = coord_x;
    Target_y = coord_y;
    Target_Orientation = orientation;
    Target_type = type;
    april_id = id;
}

GoalBased::~GoalBased()
{
    ROS_INFO("[DTor]: So not Based.");
}

void GoalBased::actionTask()
{
    
}
// void GoalBased::actionTask(TurtleBot3* bot)
// {
//     ROS_INFO("Resupplying rossenheimer");
    
// }