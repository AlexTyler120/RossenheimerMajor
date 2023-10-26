// Header Guard
#ifndef LIDAR_H
#define LIDAR_H

// Include libraries
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Include the parent class header file
#include "Sensor.h"
#define DEG2RAD (M_PI / 180.0)

//---Lidar Implementation------------------------------------------------------------
// Lidar is a child class of the Sensor class. It possesses the same functions as
// defined in the parent class, but contains an array of angles, lidar distances,
// and desired distances to the left wall (when left wall following), front wall (when
// detecting a wall), and the escape range - the degree to which the TurtleBot3 rotates
// when it has found a wall.
class Lidar: public Sensor
{
   public:
       //--Constructors and Destructors--
       Lidar();
       Lidar(ros::NodeHandle& nh_);
       ~Lidar();

       //--Functionality--

       // sensorCallBack redefines the virtual function from the abstract parent class
       // This function, uses the
       void sensorCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
       double sensorGetData(int req);
       void sensorSetData(double req);

   private:

       ros::Subscriber laser_scan_sub_;

       // array of angles to scan
       double sensor_angles_[6] = {0.0, 0.0, 0.0, 0.0, 0.0};

       // array to store scanned data
       double scan_data_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

       // member variables for lidar
       double escape_range_;
       double check_side_dist_;
       double check_forward_dist_;
      
};


#endif
