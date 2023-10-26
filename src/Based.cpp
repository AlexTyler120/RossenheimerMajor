#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalBased.h"

GoalBased::GoalBased(double coord_x, double coord_y, double pose_z, double pose_w, int type, int id)
{
    ROS_INFO("[CTor]: Based");
    ROS_INFO("OH SHIT GOAL CREATED OH NO.");
    ROS_INFO("Goal Created");
    status = true;

    Target_x = coord_x;
    Target_y = coord_y;
    Pose_z = pose_z;
    Pose_w = pose_w;
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