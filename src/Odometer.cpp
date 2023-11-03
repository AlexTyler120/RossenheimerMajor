// Include libraries
#include <ros/ros.h>

// Include class headers
#include "../include/Rossenheimer/Sensors/Sensor.h"
#include "../include/Rossenheimer/Sensors/Odometer.h"

//---Odometer Functionality------------------------------------------------------------

// Odometer Constructor and Destructor functions
Odometer::Odometer()
{
    std::cout << "[CTor]: Odometer" << std::endl;
}
Odometer::Odometer(ros::NodeHandle& nh_)
{
    // Creating subscriberes and publishers
    odom_sub_ = nh_.subscribe("odom", 10, &Odometer::sensorCallBack, this);
    position_pub_ = nh_.advertise<geometry_msgs::Point>("robot_position", 10);
    orientation_pub = nh_.advertise<geometry_msgs::Quaternion>("robot_orientation", 10);

    // initial pose positions
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
    odom_z = msg->pose.pose.position.z;

    pose_x = msg->pose.pose.orientation.x;
    pose_y = msg->pose.pose.orientation.y;
    pose_z = msg->pose.pose.orientation.z;
    pose_w = msg->pose.pose.orientation.w;

	tb3_pose_= atan2(siny, cosy);

    geometry_msgs::Point position;
    position.x = odom_x;
    position.y = odom_y;
    position.z = odom_z;
    
    geometry_msgs::Quaternion orientation;
    orientation.x = pose_x;
    orientation.y = pose_y;
    orientation.z = pose_z;
    orientation.w = pose_w;

    // Publishing position and orientation
    position_pub_.publish(position);
    orientation_pub.publish(orientation);
    
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
            break;
        case 1:
            return tb3_pose_;
            break;
        case 2:
            return odom_x;
            break;
        case 3: 
            return odom_y;
            break;
        case 4:
            return odom_z;
            break;
        case 5:
            return pose_x;
            break;
        case 6:
            return pose_y;
            break;
        case 7:
            return pose_z;
            break;
        case 8:
            return pose_w;
            break;
        default:
            return 0.0;
    }
}