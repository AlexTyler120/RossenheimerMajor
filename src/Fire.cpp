#include <ros/ros.h>

#include "../include/Rossenheimer/Goal/GoalFire.h"

GoalFire::GoalFire(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id)
{
    ROS_INFO("[CTor]: Fire.");
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


GoalFire::~GoalFire()
{
    ROS_INFO("[DTor]:Fire Extinguished");
} 


void GoalFire::actionTask()
{
    ROS_INFO("Extinguishing Fire");
    // status = false;
}