#include <ros/ros.h>

#include "Rossenheimer/Goal/GoalFire.h"

GoalFire::GoalFire(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("[CTor]: Fire ablaze");
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