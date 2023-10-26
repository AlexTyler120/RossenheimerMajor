// Header Guard
#ifndef ODOMETER_H
#define ODOMETER_H


// Include libraries
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovariance.h>

// Include the parent class header file
#include "Sensor.h"

//---Odometer Implementation------------------------------------------------------------
// Odometer is a child class of the Sensor class. It possesses the same functions as
// defined in the parent class, but contains doubles of its current pose and its previous
// pose. It is able to call, get and set sensor data that observes the position of the
// turtlebot.

class Odometer: public Sensor
{
    public:
        //--Constructors and Destructors--
        Odometer();
        Odometer(ros::NodeHandle& nh_);
        ~Odometer();

        //--Functionality--


        // sensorCallBack redefines the virtual function from the abstract parent class
        // This function allows for the turtlebot to know its current pose
        void sensorCallBack(const nav_msgs::Odometry::ConstPtr &msg);

        // sensorGetData redefines the virtual function from the abstract parent class
        // This function returns either the previous or current position of the turtlebot
        // depending on the given flag 'req' (0, 1 respectively)
        double sensorGetData(int req);

        // sensorSetData redefines the virtual function from the abstract parent class
        // This function updates the previous pose of the turtlebot (prev_tb3_pose) to the
        // current position (pose)
        void sensorSetData(double pose);

        double getOdom_x();
        double getOdom_y();

    private:
        ros::Subscriber odom_sub_;
        ros::Publisher position_pub_;
        double tb3_pose_;
        double prev_tb3_pose_;

        double odom_x;
        double odom_y;
        double pose_z;
        double pose_w;
        
};

#endif
