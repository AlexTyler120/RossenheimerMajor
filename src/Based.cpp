#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalBased.h"

GoalBased::GoalBased(double coord_x, double coord_y, double orientation, int type, int id)
{
   ROS_INFO("[CTor]: Based");
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