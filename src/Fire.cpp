// Includes
#include <ros/ros.h>

// Include header files
#include "../include/Rossenheimer/Goal/GoalFire.h"

//--Fire Object Constructor--
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

//--Fire Object Destructor--
GoalFire::~GoalFire()
{
    ROS_INFO("[DTor]:Fire Extinguished");
} 

//actionTask used to extinguish the fire when the goal is reached
void GoalFire::actionTask()
{
    ROS_INFO("Extinguishing Fire");
}
