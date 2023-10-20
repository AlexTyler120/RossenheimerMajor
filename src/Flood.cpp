#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalFlood.h"

GoalFlood::GoalFlood(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("[CTor]:Flood rushing");
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