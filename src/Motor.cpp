// Include header file for Motor class
#include "../include/Rossenheimer/Motor.h"

// Include packages
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//--Motor Default Constructor--
Motor::Motor()
{
    std::cout << "[CTor]: Motor" << std::endl;
}

//--Motor Constructor with Node Handle--
Motor::Motor(ros::NodeHandle& nh_)
{
    std::cout << "[CTor]: Motor" << std::endl;

    // establish name of topic for publishing command velocity
    std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

    // advertise the publisher for command velocity
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
}

//--Motor Destructor--
Motor::~Motor()
{
    std::cout << "[DTor]: Motor" << std::endl;
    updateCommandVelocity(0.0, 0.0);        // set velocities to 0.
}

//--Motor updateCommandVelocity--
// This function takes in two double type values, and assigns their values
// to the linear and angular velocity of the TurtleBot3
void Motor::updateCommandVelocity(double linear, double angular)
{
    
    geometry_msgs::Twist cmd_vel;   // Access command velocity from geometry_msgs

    cmd_vel.linear.x  = linear;     // Access linear velocity
    cmd_vel.angular.z = angular;    // Access angular velocity
    cmd_vel_pub_.publish(cmd_vel);  // Publish command velocities
}
