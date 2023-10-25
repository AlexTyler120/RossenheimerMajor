// Header guard
#ifndef CONTROLLER_H
#define CONTROLLER_H


// Include ros
#include <ros/ros.h>


// Include all necessary header files
#include "Sensors/Sensor.h"
#include "Sensors/Lidar.h"
#include "Sensors/Odometer.h"
#include "Motor.h"
#include "Camera.h"
#include "Navigator.h"


// enum:case structure for
enum
{   GET_TB3_DIRECTION = 0,
    TB3_FIND_WALL,
    TB3_RIGHT_TURN,
    TB3_WALL_FOLLOW,
    TB3_APRIL_CENTER,
    TB3_STOP
};


//---Controller Implementation---------------------------------------------------------------
// The Controller of the TurtleBot3 is used to implement the desired maze solving algorithm
// for the TurtleBot3. It uses an 'association' relationship with the Sensor and the Motor of the
// TurtleBot3 to enact the relevant manoeuvres according to the case it is in, defined by the enum
// above.
class Controller
{
  
   public:
    //--Constructor--
    Controller(); // default constructor

    //--Destructor--
    ~Controller(); // default destructor
    //--Functionality--
    void MazeSolver(Sensor* readLidar, Sensor* readOdometer, Motor* readMotor, Camera* readCamera);
      


   private:
    //--Functionality--
    
    // findWall sets the TurtleBot3 on an arc trajectory leftward until a wall is found
    void findWall(Motor* readMotor);


    // rightTurn turns the TurtleBot3 right until its pose is parallel to the left wall
    void rightTurn(Sensor* readOdometer, Sensor* readLidar, Motor* readMotor);


    // wallFollow continuously manipulates the angular velocity of the TurtleBot3
    // to maintain the desired distance from the left wall, and direction parallel to
    // the left wall
    void wallFollow(double left_distance, Sensor* readLidar, Sensor* readOdometer, Motor* readMotor);


    void centerTag(Camera* readCamera, Motor* readMotor, Sensor* readLidar);

    Navigator* _Navigator; // controller has a navigator

    //--Private Member Variables--
    uint8_t turtlebot3_state_num;        // variable to store case for algorithm

    //--Constant Definitions--
    double PROXIMITY        = 0.4; // prev 0.2
    double WALL_LINEAR      = 0.12;
    double WALL_ANGULAR     = 0.5;
    double STATIONARY       = 0.0;
    double RIGHT_ANGULAR    = -0.9; //prev -2.9
    double OVERALL_LIMIT    = 0.17;
    double DIST_LIMIT_1     = 0.003;
    double DIST_LIMIT_2     = 0.03;
    double FOLLOW_LINEAR    = 0.1; //prev 1.5
    double FOLLOW_ANGULAR   = 0.7; //prev 1.5

    int MULTIPLIER_1 = 10;
    int MULTIPLIER_2 = 20;
    int MULTIPLIER_3 = 30;

    double MULTIPLIER_LIMIT = 0.7;

    double FLAG_IN_CENTER  = 30.0;
};


#endif