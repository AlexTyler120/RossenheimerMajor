#include <ros/ros.h>

#include "Rossenheimer/Goal/GoalBased.h"

GoalBased::GoalBased()
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