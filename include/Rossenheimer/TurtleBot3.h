// Header Guard
#ifndef TB3_
#define TB3_


// iIncludes
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// Include headers
#include "Sensors/Sensor.h"
#include "Motor.h"
#include "Controller.h"
#include "Camera.h"

//Odom reading values
#define ODOM_X_INDEX 2
#define ODOM_Y_INDEX 3

//Forward declaration
class Controller;
class Navigator;
class Goal;


//---TurtleBot3 Implementation-------------------------------------
// TurtleBot3 is a class which resembles TurtleBot3 Burger Bot
// in an object oriented environment. It possesses (inherits) the
// component classes: Sensors (2 of them), a motor, a camera and a controller
//
class TurtleBot3
{
   public:
          //--Constructors and Destructors--
          TurtleBot3(ros::NodeHandle& nh_);
          ~TurtleBot3();


          //--Functionality--
          // extinguish_world utilises the member _controller in the TurtleBot3
          // to run the goal setting and reaching algorithm, and run necessary
          // maneuvers and movements, depending on readings from the
          // Sensors.
          bool ExtinguishWorld();

          void AddPathPoint();       // add path point to vector
          double GetOdom(int req);   // get odometer data

          void UpdateTank(int* depot_array);

   private:
          //-- Consts for the TurtleBot3 
          static const uint8_t LIDAR_INDEX = 0;       // index for Lidar Sensor
          static const uint8_t ODOMETER_INDEX = 1;    // index for Odometer Sensor
          static const uint8_t MAX_SENSORS = 2;       // max number of TurtleBot3 sensors

          //--Member Variables--
          int sandbag_payload;
          int spray_payload;

          Sensor* TurtleBotSensors[MAX_SENSORS];      // array of Sensors pointers (child: Lidar, Odometer)
          Motor* _motor;                              // pointer to motor (manipulate linear/angular velocity)
          Controller* _controller;                    // pointer to controller (brain/ algorithm of TurtleBot3)
          Camera* _camera;

          ros::Publisher marker_pub;                           //publisher for path
          visualization_msgs::Marker path_marker;
};


#endif

