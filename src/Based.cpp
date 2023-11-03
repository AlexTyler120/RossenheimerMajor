// Includes
#include <ros/ros.h>

// Include header files
#include "../include/Rossenheimer/Goal/GoalBased.h"

//--Based Object Constructor--
GoalBased::GoalBased(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id)
{
    ROS_INFO("[CTor]: Based");
    ROS_INFO("OH SHIT GOAL CREATED OH NO.");
    ROS_INFO("Goal Created");
    status = true;

    Target_x = px;
    Target_y = py;
    Target_z = pz;
    Pose_x = ox;
    Pose_y = oy;
    Pose_z = oz;
    Pose_w = ow;
    
    Target_type = type;
    april_id = id;
}

//--Based Object Destrucctor--
GoalBased::~GoalBased()
{
    ROS_INFO("[DTor]: So not Based.");
}

//actionTask used to resupply the TurtleBot
void GoalBased::actionTask()
{
    ROS_INFO("Resupplying rossenheimer");
}
