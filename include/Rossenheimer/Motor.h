// Header Guard
#ifndef MOTOR_H_
#define MOTOR_H_

// Include libraries
#include <ros/ros.h>


//---Motor Implementation------------------------------------------------------------
// Motor is a class that controls the motion of the turtlebot, in both linear and
// angular directions. It handles the ability for the turtlebot to adapt based on
// information from its sensors and logic.

class Motor
{
    public:
        //--Constructors and Destructors--
        Motor();
        Motor(ros::NodeHandle* nh_);
        ~Motor();

        //--Functionality--

        // updateCommandVelocity takes numerical inputs to control the linear
        // and angular speed of the turtlebot. It is the primary controller
        // of the velocity of the turtlebot throughout the program
        void updateCommandVelocity(double linear, double angular);
    private:
        ros::Publisher cmd_vel_pub_;    

};

#endif
