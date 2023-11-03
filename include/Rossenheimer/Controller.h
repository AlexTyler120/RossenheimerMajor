// Header guard
#ifndef CONTROLLER_H
#define CONTROLLER_H



#include <ros/ros.h>                    // include ros
#include <nav_msgs/OccupancyGrid.h>     // include for geometry
#include <std_msgs/Float32.h>           // include message type
#include <cmath>                        // include math


// Include all necessary header files
#include "Sensors/Sensor.h"
#include "Sensors/Lidar.h"
#include "Sensors/Odometer.h"
#include "Motor.h"
#include "Camera.h"
#include "Navigator.h"



// case structure for each objective in project

enum
{
    TB3_FRONTIER_DETECTION = 0,
    TB3_MOVE_BASE,
    TB3_MOVE_GOAL,
    TB3_END
};


//---Controller Implementation---------------------------------------------------------------
// The Controller of the TurtleBot3 behaves as the brains of the TurtleBot3
// It uses the enums above in a switch case, in which, it determines whether
// the TB3 should be frontier detecting, moving to base (to resupply), moving
// to the next urgent incident, or returning to base once all fires and floods 
// have been halted.

class Controller
{
  
   public:
    //--Constructor--
    Controller(Sensor* readOdometer, ros::NodeHandle& nh_); // default constructo

    //--Destructor--
    ~Controller(); // default destructor

    //--Functionality--

    // SaveWorld uses the enums and switch case structure to determine what series of actions
    // the TB3 should be performing. It is the central function of the class, from which other
    // supporting functions are called to achieve the TB3's current objective.
    void SaveWorld(Sensor* readLidar, Sensor* readOdometer, Motor* readMotor, Camera* readCamera);
    
    
   private:
    //--Functionality--
    
    
    // FrontierDetection utilises position and orientation data from the Odometer, and april tag
    // data from the Camera in order to log all incidents (fires and floods) in the environment
    // including type (fire or flood), priority (0, 1, or 2) and location of TB3 when incident
    // encountered.
    void FrontierDetection(bool tag_detected, double tag_offset, int tagID, double px, double py, double pz, double ox, double oy, double oz, double ow);
    
    
    // CenterTag detects flags in the environment, and approaches them from the front.
    // It uses data from the Camera, and Lidar to detect the flag, and then manipulates
    // the velocity of TB3 through the Motor. 
    void CenterTag(Camera* readCamera, Motor* readMotor, Sensor* readLidar);

    //-- Controller inherits a Navigator.
    // This Navigator enables Controller to achieve its objectives, using 
    // move_base (movement to incidents) and geometry_msgs (storing position
    // and orientation).
    Navigator* _Navigator; // controller has a navigator


    //--Private Member Variables--

    uint8_t turtlebot3_state_num;        // variable to store case for SaveWorld()

    double timer;

    int prev_tagID = -1;                // variable to store previous April Tag ID

    // constants for flag detection
    double STATIONARY       = 0.0;      // velocity = 0
    double OVERALL_LIMIT    = 0.17;     // velocity upper bound
    double FOLLOW_LINEAR    = 0.1;      // base linear velocity
    double FOLLOW_ANGULAR   = 0.7;      // base angular velocity
    double FLAG_IN_CENTER  = 30.0;      // offset of flag (height)
    double FLAG_DETECT     = 15.0;      // upper bound of offset - for detection
    double TAG_LIMIT        = 1.0;      // proximity to read tag

    // time for map exploration
    double EXPLORATION_TIME = 200.0;

};


#endif
