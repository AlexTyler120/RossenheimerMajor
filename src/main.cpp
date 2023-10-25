// This program uses Object-Oriented Design principles to control the
// movement and manoeuvres of the TurtleBot3 Burger to solve a maze.
// We utilise the strategy of left wall following to find a path to
// the desired destination. A camera node is also implemented to
// detect a finish target within the maze.


// This program is the works of:
// Alec Cook
// Rishikesh Deshpande
// Timothy Hegarty
// Alex Tyler


// Project Name: Rossenheimer
// Subtext: Now I am become turtle [DTor:] of maze.


#include <ros/ros.h>        // include ros


// include header file for TurtleBot3
#include "../include/Rossenheimer/TurtleBot3.h"



int main(int argc, char** argv) {


   ros::init(argc, argv, "turtlebot3_drive");  // initialise ros later use
   ros::NodeHandle nh_;                        // construct node handler
   TurtleBot3 Ken(nh_);                        // instantiate TurtleBot3 object

   Ken.solveMaze( &Ken);                            // call solving Maze Function
   
  
   return 0;
}
