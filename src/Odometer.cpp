// Include libraries
#include <ros/ros.h>

// Include class headers
#include "Rossenheimer/Sensors/Sensor.h"
#include "Rossenheimer/Sensors/Odometer.h"

//---Odometer Functionality------------------------------------------------------------

// Odometer Constructor and Destructor functions
Odometer::Odometer()
{
    std::cout << "[CTor]: Odometer" << std::endl;
}
Odometer::Odometer(ros::NodeHandle* nh_)
{
    odom_sub_ = nh_->subscribe("odom", 10, &Odometer::sensorCallBack, this);

    tb3_pose_ = 0.0;
    prev_tb3_pose_ = 0.0;
}

Odometer::~Odometer()
{
    std::cout << "[DTor]: Odometer" << std::endl;
}

// sensorCallBack used to know its current pose
void Odometer::sensorCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;  

	tb3_pose_= atan2(siny, cosy);
    
}

// sensorSetData used to return either previous or current pose
void Odometer::sensorSetData(double pose)
{
    std::cout << "SETTING DATA FOR PREV. POSE" << std::endl;
    prev_tb3_pose_ = pose;
    return;
}

// sensorSetData used to update previous pose
double Odometer::sensorGetData(int req){
    switch(req)
    {
        case 0:
            return prev_tb3_pose_;
        case 1:
            return tb3_pose_;
        case 2:
            return odom_x;
        case 3: 
            return odom_y;
        default:
            return 0.0;
    }
}