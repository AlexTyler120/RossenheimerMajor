// Header Guard
#ifndef SENSORS_H_
#define SENSORS_H_


// Includes
#include <ros/ros.h>                // include ros
#include <ros/message_traits.h>     // include message_traits.h
#include <sensor_msgs/LaserScan.h>  // msg type for Lidar subscribe
#include <geometry_msgs/Twist.h>    // msg type for Motor publish
#include <nav_msgs/Odometry.h>      // msg for odometer subscriber
#include <iostream>         

// indexes for Lidar scan data
enum
{
    CENTER = 0,
    LEFT,
    RIGHT,
    FRONTLEFT,
    BACKLEFT
};


//---Sensor Implementation---------------------------------------------------------
// The Sensor class is parent to the child classes: Lidar and Odometer
// It posses virtual and pure virtual functions that are redefined
// used by the child classes, in accordance with the member data local
// to each child class.


class Sensor
{
   public:
       //--Constructors and Destructors--
       Sensor();
       Sensor(ros::NodeHandle& nh_);
       virtual ~Sensor();      // virtual destructor - redefined by child classes


       //--Functionality--
       // virtual function for subscribing to node
       virtual void sensorCallBack(); 


       // pure virtual function for returning data pertaining to sensor
       virtual double sensorGetData(int req) = 0; 


       // virtual function for setting member data variables
       virtual void sensorSetData(double req);
};


#endif
