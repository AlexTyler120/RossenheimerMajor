// This program uses Object-Oriented Design principles to control the
// movement and manoeuvres of the TurtleBot3 Burger to identify points of interest and relocate them.
/*We utilise frontier detection to completely map out its area and during this process save points of interest
  with the camera using april tags that the turtlebot can later return to, resolving the disaster*/


// This program is the works of:
// Alec Cook
// Rishikesh Deshpande
// Timothy Hegarty
// Alex Tyler


// Project Name: Rossenheimer Major
// Subtext: Now I am become turtle [DTor:] of fire and flood.


#include <ros/ros.h>        // include ros


// include header file for TurtleBot3
#include "../include/Rossenheimer/TurtleBot3.h"



int main(int argc, char** argv) {

   ROS_INFO("MAIN HERE.");

   ros::init(argc, argv, "Rossenheimer");     // initialise ros
   ros::NodeHandle nh_;                       // construct node handler
   TurtleBot3 Ken(nh_);                       // instantiate TurtleBot3 object and pass in node handler

   Ken.ExtinguishWorld();                    //execute program
   
  
   return 0;
}
