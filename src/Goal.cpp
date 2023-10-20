#include <ros/ros.h>
#include "Rossenheimer/Goal/Goal.h"

Goal::Goal(){
    ROS_INFO("OH SHIT GOAL CREATED OH NO.");    
}

Goal::Goal(double coord_x, double coord_y, double orientation)
{
    ROS_INFO("Goal Created");
    status = true;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = coord_x;
    goal.target_pose.pose.position.y = coord_y;
    goal.target_pose.pose.orientation.w = orientation;
}

Goal::~Goal(){
    ROS_INFO("OH SHIT GOAL DESTROYED OH YEAH.");
}

void Goal::actionTask(){

}
