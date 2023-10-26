#include <ros/ros.h>
#include "../include/Rossenheimer/Goal/Goal.h"

Goal::Goal()
{
    ROS_INFO("Goal Created.");
    
}
Goal::Goal(double coord_x, double coord_y, double pose_z, double pose_w, int type, int id){
    ROS_INFO("OH SHIT GOAL CREATED OH NO.");
    ROS_INFO("Goal Created");
    status = true;
  
    

    Target_x = coord_x;
    Target_y = coord_y;
    Pose_z = pose_z;
    Pose_w = pose_w;
    Target_type = type;
    april_id = id;
}

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
        default:
            return 0.0;

    }
}
// Goal::Goal(double coord_x, double coord_y, double orientation)
// {
//     ROS_INFO("Goal Created");
//     status = true;
  
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.header.stamp = ros::Time::now();
    
//     goal.target_pose.pose.position.x = coord_x;
//     goal.target_pose.pose.position.y = coord_y;
//     goal.target_pose.pose.orientation.w = orientation;
// }

Goal::~Goal(){
    ROS_INFO("OH SHIT GOAL DESTROYED OH YEAH.");
}

int Goal::GetType()
{
    return Target_type;
}
void Goal::actionTask(){

}
