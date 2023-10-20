#include <ros/ros.h>
#include "../include/Rossenheimer/Goal/Goal.h"

Goal::Goal(){
    ROS_INFO("OH SHIT GOAL CREATED OH NO.");    
}

Goal::Goal(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("Goal Created");
    status = true;
    Target_x = coord_x;
    Target_y = coord_y;
    Target_Orientation = orientation;    
}

Goal::~Goal(){
    ROS_INFO("OH SHIT GOAL DESTROYED OH YEAH.");
}

void Goal::actionTask(){

}
