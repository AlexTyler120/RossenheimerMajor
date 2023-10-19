#include <ros/ros.h>

#include "../include/Rossenheimer/Controller.h"
#include "../include/Rossenheimer/Sensor.h"

//-- Maze Solver Function --//
// This function will check if there is a wall to the left or infront and will take action accordingly
void Controller::MazeSolver(Sensor* readLidar, Sensor* readOdometer, Motor* readMotor, Camera* readCamera)
{
    std::cout << "TESTING MAZE SOLVER" << std::endl;
    turtlebot3_state_num = GET_TB3_DIRECTION;

    bool left_wall = readLidar->sensorGetData(LEFT) < PROXIMITY; //there is a wall to the left 0.3m away

    bool front_wall = readLidar->sensorGetData(CENTER) < PROXIMITY; //there is a wall in front 0.3m away

    //get the difference the front left and back left lidar readings to determine paralellism
    long double left_distance = readLidar->sensorGetData(FRONTLEFT) - readLidar->sensorGetData(BACKLEFT);

    bool tag_detected = readCamera->getTagDetected(); // reading in if a tag is detected

    // reading in distance from april tag to center of camera
    double tag_offset = readCamera->getTagOffset();


    if (tag_detected)
    {
      turtlebot3_state_num = TB3_APRIL_CENTER;
    }
    else if (!left_wall && !front_wall) //no walls are seen so go forward and left slightly looking for one
    {
      turtlebot3_state_num = TB3_FIND_WALL;
    }
    else if (left_wall && !front_wall) // theres a wall to the left so follow it, might be following on a diagonal so will use too_close to adjust
    {
      turtlebot3_state_num = TB3_WALL_FOLLOW;
    }
    else //if there is a wall in front at all make a right turn
    {
      turtlebot3_state_num = TB3_RIGHT_TURN;
    }



    switch(turtlebot3_state_num){ //switch case to determine current state of the turtlebot

      case TB3_FIND_WALL: //Wall finding state (veering left)
        std::cout << "FINDING WALL" << std::endl;
        findWall(readMotor);
        break;

      case TB3_RIGHT_TURN: //turning right state until parallel to wall
        std::cout << "RIGHT TURN" << std::endl;
        readOdometer->sensorSetData(readOdometer->sensorGetData(1));
        rightTurn(readOdometer, readLidar, readMotor);
        break;

      case TB3_WALL_FOLLOW: //following wall state to keep near parallel 
        std::cout << "FOLLOWING WALL" << std::endl;
        wallFollow(left_distance, readLidar, readOdometer, readMotor);
        break;

      case TB3_APRIL_CENTER:
        std::cout << "CENTERING" << std::endl;
        centerTag(readCamera, readMotor);
        break;

      case TB3_STOP:
        std::cout << "STOPPING" << std::endl;
        readMotor->updateCommandVelocity(STATIONARY, STATIONARY);

      default:
        std::cout << "GETTING TB3 DIRECTION" << std::endl;
        break;
    }

    return;

}
// findWall sets the TurtleBot3 on an arc trajectory leftward until a wall is found
void Controller::findWall(Motor* readMotor)
{
    readMotor->updateCommandVelocity(WALL_LINEAR, WALL_ANGULAR);
}
// rightTurn turns the TurtleBot3 right until it hits the escape range where it adjusts as it follows the wall
void Controller::rightTurn(Sensor* readOdometer, Sensor* readLidar, Motor* readMotor)
{
    //if the difference between the current pose and the previous pose is greater than the escape range turning done
   if (fabs(readOdometer->sensorGetData(0) - readOdometer->sensorGetData(1)) < readLidar->sensorGetData(-1))  
   {
    readMotor->updateCommandVelocity(STATIONARY, RIGHT_ANGULAR);
   } 
   return;
}

// wallFollow continuously manipulates the angular velocity of the TurtleBot3
void Controller::wallFollow(double left_distance, Sensor* readLidar, Sensor* readOdometer, Motor* readMotor)
{
    int multiplier;
    //if the distance is greater than 0.17m away from the wall use the normal angular velocity-
    if (readOdometer->sensorGetData(1) > OVERALL_LIMIT)
    {
        if (fabs(left_distance) < DIST_LIMIT_1) 
        {
          multiplier = MULTIPLIER_1;
        }
        //same here, little adjustment needed if distance is 0.03 will be 0.3 angular velocity again
        else if (fabs(left_distance) < DIST_LIMIT_2)
        {
          multiplier = MULTIPLIER_2;
        }
        else //adjust it by the distance value
        {
          multiplier = MULTIPLIER_3;
        }
    }
    //if the distance is less than 0.2m away from the wall use the too close angular velocity
    else {
        multiplier = MULTIPLIER_1;
    }
    //if the distance is 0 then the angular velocity is 0 no need to turn
    if (left_distance == 0)
    {
        multiplier = STATIONARY;
    }

    if (fabs(left_distance*multiplier < MULTIPLIER_LIMIT))
    {
      readMotor->updateCommandVelocity(FOLLOW_LINEAR, left_distance*multiplier);
    }
    else
    {
      readMotor->updateCommandVelocity(FOLLOW_LINEAR, FOLLOW_ANGULAR);
    }

    return;
}


void Controller::centerTag(Camera* readCamera, Motor* readMotor)
{
    //if the flag is in the centre of the camera approach
    if ((readCamera->getTagOffset() < FLAG_IN_CENTER) && (readCamera->getTagOffset() > -FLAG_IN_CENTER))
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, STATIONARY);
      ROS_INFO("Tag in center");
    }
    //if the flag is to the right of the camera turn right
    else if (readCamera->getTagOffset() > FLAG_IN_CENTER)
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, -FOLLOW_ANGULAR);
    }
    //if the flag is to the right of the camera turn right
    else if (readCamera->getTagOffset() < -FLAG_IN_CENTER)
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, FOLLOW_ANGULAR);
    }
  return;
}
