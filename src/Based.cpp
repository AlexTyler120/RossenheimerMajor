#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalBased.h"

Base::Base()
{
    ROS_INFO("[CTor]: Based");
}

Base::~Base(){
    ROS_INFO("[DTor]: So not Based.");
}

void Base::actionTask()
{
    ROS_INFO("Resupplying rossenheimer");
    status = false;
}