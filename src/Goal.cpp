// Includes
#include <ros/ros.h>

// Include header files
#include "../include/Rossenheimer/Goal/Goal.h"

//--Goal Constructor ----------------------------
Goal::Goal()
{
    ROS_INFO("Goal Created.");
}

//--Goal Object Constructor ----------------------------
Goal::Goal(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id){
    ROS_INFO("Goal Created");
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

// GetPosition used to return a desired index of goal position/pose
double Goal::GetPosition(int req)
{
    switch(req)
    {
        case 0:
            return Target_x;
        case 1:
            return Target_y;
        case 2:
            return Target_z;
        case 3:
            return Pose_x;
        case 4:
            return Pose_y;
        case 5:
            return Pose_z;
        case 6:
            return Pose_w;
        case 7:
            return Target_type;
        case 8:
            return april_id;
        default:
            return 0.0;

    }
}

//--Destructor--
Goal::~Goal(){
    
}

// GetType used to return the type of goal
int Goal::GetType()
{
    return Target_type;
}

//ActionTask is a pure virtual function redefined by child classes
void Goal::actionTask(){

}
