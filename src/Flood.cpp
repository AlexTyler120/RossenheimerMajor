#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalFlood.h"

GoalFlood::GoalFlood(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id)
{
    ROS_INFO("[CTor]: Flood created.");
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

GoalFlood::~GoalFlood()
{
    ROS_INFO("[DTor]: Flood blocked");
} 


void GoalFlood::actionTask()
{
    ROS_INFO("blocking Flood");
    status = false;
}