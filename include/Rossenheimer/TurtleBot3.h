// Header Guard
#ifndef TB3_
#define TB3_


// including ros package
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// include necessary header files
#include "Sensors/Sensor.h"
#include "Motor.h"
#include "Controller.h"
#include "Camera.h"

#define ODOM_X_INDEX 2
#define ODOM_Y_INDEX 3

//---TurtleBot3 Implementation-------------------------------------
// TurtleBot3 is a class which resembles TurtleBot3 Burger Bot
// in an object oriented environment. It possesses (inherits) the
// component classes: Sensors (2 of them), a motor, and a controller
// (where the algorithm for maze solving is determined).
//
class TurtleBot3
{
   public:
        //--Constructors and Destructors--
        TurtleBot3(ros::NodeHandle &nh_);
        ~TurtleBot3();


        //--Functionality--
        // solveMaze utilises the member _controller in the TurtleBot3
        // to run the maze solving algorithm, and run necessary
        // manoeuvres and movements, depending on readings from the
        // Sensors.
        bool solveMaze();

        void addPathPoint();       // add path point to vector
        double getOdom(int req);   // get odometer data

   private:
        //-- Consts for the TurtleBot3 
        static const uint8_t LIDAR_INDEX = 0;       // index for Lidar Sensor
        static const uint8_t ODOMETER_INDEX = 1;    // index for Odometer Sensor
        static const uint8_t MAX_SENSORS = 2;       // max number of TurtleBot3 sensors

        //--Member Variables--
        Sensor* TurtleBotSensors[MAX_SENSORS];      // array of Sensors pointers (child: Lidar, Odometer)
        Motor* _motor;                              // pointer to motor (manipulate linear/angular velocity)
        Controller* _controller;                    // pointer to controller (brain/ algorithm of TurtleBot3)
        Camera* _camera;

        ros::Publisher marker_pub;                           // publisher for path
        visualization_msgs::Marker path_marker;
};


#endif

