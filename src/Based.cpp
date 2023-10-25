#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalBased.h"

GoalBased::GoalBased(double coord_x, double coord_y, double orientation, int type)
{
   ROS_INFO("[CTor]: Based");
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