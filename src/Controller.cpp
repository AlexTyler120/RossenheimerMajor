#include <ros/ros.h>

#include "../include/Rossenheimer/Controller.h"

Controller::Controller(Sensor* readOdometer, ros::NodeHandle& nh_)
{
    ROS_INFO("[CTor]: Controller");
    _Navigator = new Navigator(nh_);
    // Sensor* InitPose;
    int base_april = 0;
    ROS_INFO("MADE IT HERE.");
    // _Navigator->SetBase(readOdometer->sensorGetData(2), readOdometer->sensorGetData(3), readOdometer->sensorGetData(1), );
    _Navigator->SetBase(-1.89, 0.1077, 0, 1, TYPE_BASE, 0);
    count = 0;
    turtlebot3_state_num = TB3_MOVE_BASE;
}

Controller::~Controller()
{
    ROS_INFO("[DTor]: Controller");
    delete _Navigator;
}

void Controller::frontierDetection(Camera* readCamera, Sensor* readLidar, Sensor* readOdometer)
{
    // ROS_INFO("Count: %d", count);
    double current_x;
    double current_y;
    double current_pose;

    ROS_INFO("5s elapsed: setting previous.");
    prev_x = readOdometer->sensorGetData(2);
    prev_y = readOdometer->sensorGetData(3);

    ROS_INFO("%f, %f", prev_x, prev_y);
    ros::Rate rate(0.25);
    rate.sleep();

    if (fabs(readOdometer->sensorGetData(2) - prev_x) < 0.1 && fabs(readOdometer->sensorGetData(3) - prev_y) < 0.1)
    {
      ROS_INFO("NOT MOVING, DISTANCE: %f, %f",fabs(readOdometer->sensorGetData(2) - prev_x), fabs(readOdometer->sensorGetData(3) - prev_y));
    }
    else
    {
      ROS_INFO("MOVING, DISTANCE: %f, %f",fabs(readOdometer->sensorGetData(2) - prev_x), fabs(readOdometer->sensorGetData(3) - prev_y));
    }
    
      bool tag_detected = readCamera->getTagDetected(); // reading in if a tag is detected

      // reading in distance from april tag to center of camera
      double tag_offset = readCamera->getTagOffset();

      double odom_x = readOdometer->sensorGetData(2);   // TODO: fix floating numbers, define in .h and remove definition in TurtleBot.h
      double odom_y = readOdometer->sensorGetData(3);
      double orientation = readOdometer->sensorGetData(1);

      // ROS_INFO("x: %f, y: %f, pose: %f", odom_x, odom_y, orientation);
    
      if (tag_detected && ((readCamera->getTagOffset() < 15) && (readCamera->getTagOffset() > -15)))  // TODO: fix magic numbers
      {
        int tag_ID = readCamera->getTagID();
        double odom_x = readOdometer->sensorGetData(2);   // TODO: fix floating numbers, define in .h and remove definition in TurtleBot.h
        double odom_y = readOdometer->sensorGetData(3);
        double orientation = readOdometer->sensorGetData(1);

        _Navigator->SetGoal(tag_ID, odom_x, odom_y, orientation);
        
        // if (tag_ID != prev_tag_ID)
        
        // {
        //   odom_saved = false;
        //   prev_tag_ID = tag_ID;
        // }
        
        // auto it = tag_positions.find(tag_ID);

        // if (!odom_saved)
        // {
        //   if (it == tag_positions.end())
        //   {
        //     tag_positions[tag_ID] = std::make_pair(odom_x, odom_y);
        //     odom_saved = true;

        //     // Print the tag ID and stored values for debugging
        //     ROS_INFO("Tag ID %d: odom_x = %f, odom_y = %f", tag_ID, odom_x, odom_y);
        //   }
        // }
        // if (it != tag_positions.end())
        // {
        //   // If the tag ID is found, retrieve the stored values
        //   double map_odom_x = it->second.first;
        //   double map_odom_y = it->second.second;

        //   // Print the tag ID and stored values for debugging
        //   ROS_INFO("Tag ID %d: odom_x = %f, odom_y = %f", tag_ID, map_odom_x, map_odom_y);
        // }
        // else
        // {
        //   // If the tag ID is not found, print an error message
        //   ROS_INFO("Tag ID %d not found in tag_positions map", tag_ID);
        // }
      }
    
    ROS_INFO("There's something inside you");
    return;
  }

void Controller::SaveWorld(Sensor* readLidar, Sensor* readOdometer, Motor* readMotor, Camera* readCamera)
{

  ROS_INFO("%f, %f, %f", readOdometer->sensorGetData(2), readOdometer->sensorGetData(3), readOdometer->sensorGetData(1));
  switch(turtlebot3_state_num)
  {
    case TB3_MOVE_BASE:
      // while(fabs(readOdometer->sensorGetData(2) - _Navigator->GetBase()->GetPosition(0) > 0.05 && fabs(readOdometer->sensorGetData(3) - _Navigator->GetBase()->GetPosition(1)) > 0.05))
      // {
      //   _Navigator->MoveToGoal(_Navigator->GetBase());
      // }
      // depot = _Navigator->algorithm();
      // turtlebot3_state_num = TB3_MOVE_GOAL;
      // while(true)
      // {

      // }
      break;

    case TB3_MOVE_GOAL:
      while (_Navigator->GetAddress().size() > 0)
      {
        _Navigator->MoveToGoal(_Navigator->GetAddress().front());
        _Navigator->GetAddress().front()->actionTask();
        _Navigator->GetAddress().erase(_Navigator->GetAddress().begin());
      }
      turtlebot3_state_num = TB3_MOVE_BASE;
      break;
  }
}


//-- Maze Solver Function --//
// This function will check if there is a wall to the left or infront and will take action accordingly
void Controller::MazeSolver(Sensor* readLidar, Sensor* readOdometer, Motor* readMotor, Camera* readCamera)
{
    // std::cout << "TESTING MAZE SOLVER" << std::endl;
    // ROS_INFO("%f, %f, %f ", readOdometer->sensorGetData(2), readOdometer->sensorGetData(3), readOdometer->sensorGetData(1));
    turtlebot3_state_num = GET_TB3_DIRECTION;

    bool left_wall = readLidar->sensorGetData(LEFT) < PROXIMITY; //there is a wall to the left 0.3m away

    bool front_wall = readLidar->sensorGetData(CENTER) < PROXIMITY; //there is a wall in front 0.3m away

    //get the difference the front left and back left lidar readings to determine paralellism
    long double left_distance = readLidar->sensorGetData(FRONTLEFT) - readLidar->sensorGetData(BACKLEFT);

    bool tag_detected = readCamera->getTagDetected(); // reading in if a tag is detected

    // reading in distance from april tag to center of camera
    double tag_offset = readCamera->getTagOffset();

    if (tag_detected && ((readCamera->getTagOffset() < 15) && (readCamera->getTagOffset() > -15)))  // TODO: fix magic numbers
    {
      int tag_ID = readCamera->getTagID();
      if (tag_ID != prev_tag_ID)
        {
          odom_saved = false;
          prev_tag_ID = tag_ID;
        }
      double odom_x = readOdometer->sensorGetData(2);   // TODO: fix floating numbers, define in .h and remove definition in TurtleBot.h
      double odom_y = readOdometer->sensorGetData(3);

      if (!odom_saved)
      {
        tag_positions[tag_ID] = std::make_pair(odom_x, odom_y);
        odom_saved = true;

        // Print the tag ID and stored values for debugging
        ROS_INFO("Tag ID %d: odom_x = %f, odom_y = %f", tag_ID, odom_x, odom_y);
      }
      
      // error checking, search for tag ID in the tag_positions map
      auto it = tag_positions.find(tag_ID);
      if (it != tag_positions.end())
      {
        // If the tag ID is found, retrieve the stored values
        double map_odom_x = it->second.first;
        double map_odom_y = it->second.second;

        // Print the tag ID and stored values for debugging
        ROS_INFO("Tag ID %d: odom_x = %f, odom_y = %f", tag_ID, map_odom_x, map_odom_y);
      }
      else
      {
        // If the tag ID is not found, print an error message
        ROS_INFO("Tag ID %d not found in tag_positions map", tag_ID);
      }
    }


    if (tag_detected)
    {
      turtlebot3_state_num = TB3_APRIL_CENTER;
    }
    else if (!left_wall && !front_wall) //no walls are seen so go forward and left slightly looking for one
    {
      //quick test to move to init goal
      // ROS_INFO("Moving to goal");
      // _Navigator->MoveToGoal(0);
      // ROS_INFO("Moved to goal");
      //turtlebot3_state_num = TB3_FIND_WALL;
    }
    // else if (left_wall && !front_wall) // theres a wall to the left so follow it, might be following on a diagonal so will use too_close to adjust
    // {
    //   turtlebot3_state_num = TB3_WALL_FOLLOW;
    // }
    // else //if there is a wall in front at all make a right turn
    // {
    //   turtlebot3_state_num = TB3_RIGHT_TURN;
    // }



    // switch(turtlebot3_state_num){ //switch case to determine current state of the turtlebot

    //   case TB3_FIND_WALL: //Wall finding state (veering left)
    //     std::cout << "FINDING WALL" << std::endl;
    //     findWall(readMotor);
    //     break;

    //   case TB3_RIGHT_TURN: //turning right state until parallel to wall
    //     std::cout << "RIGHT TURN" << std::endl;
    //     readOdometer->sensorSetData(readOdometer->sensorGetData(1));
    //     rightTurn(readOdometer, readLidar, readMotor);
    //     break;

    //   case TB3_WALL_FOLLOW: //following wall state to keep near parallel 
    //     std::cout << "FOLLOWING WALL" << std::endl;
    //     wallFollow(left_distance, readLidar, readOdometer, readMotor);
    //     break;

      // case TB3_APRIL_CENTER:
      //   std::cout << "CENTERING" << std::endl;
      //   centerTag(readCamera, readMotor, readLidar);
      //   break;

    //   case TB3_STOP:
    //     std::cout << "STOPPING" << std::endl;
    //     readMotor->updateCommandVelocity(STATIONARY, STATIONARY);

    //   default:
    //     std::cout << "GETTING TB3 DIRECTION" << std::endl;
    //     break;
    }

    // return;

// }
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

int* Controller::getDepots()
{
  return depot;
}
void Controller::centerTag(Camera* readCamera, Motor* readMotor, Sensor* readLidar)
{
    //if the flag is in the centre of the camera approach
    if ((readCamera->getTagOffset() < FLAG_IN_CENTER) && (readCamera->getTagOffset() > -FLAG_IN_CENTER))
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, STATIONARY);
      if ((readCamera->getTagOffset() < 15.0) && (readCamera->getTagOffset() > -15.0))
      {
        ROS_INFO("Tag in center");
        double tag_dist = readLidar->sensorGetData(CENTER);
        if (tag_dist < 1.0)
        {
          readMotor->updateCommandVelocity(0.0, 0.0);
          ROS_INFO("Stopped with tag distance: %f", tag_dist);
        }
        else if (readCamera->getTagOffset() > 15)
        {
          readMotor->updateCommandVelocity(OVERALL_LIMIT, -FOLLOW_ANGULAR);
        }
        //if the flag is to the right of the camera turn right
        else if (readCamera->getTagOffset() < -15)
        {
          readMotor->updateCommandVelocity(OVERALL_LIMIT, FOLLOW_ANGULAR);
        }
      }
      else if (readCamera->getTagOffset() > FLAG_IN_CENTER)
      {
        readMotor->updateCommandVelocity(OVERALL_LIMIT, -FOLLOW_ANGULAR);
      }
      //if the flag is to the left of the camera turn left
      else if (readCamera->getTagOffset() < -FLAG_IN_CENTER)
      {
        readMotor->updateCommandVelocity(OVERALL_LIMIT, FOLLOW_ANGULAR);
      }
    }
    //if the flag is to the right of the camera turn right
    else if (readCamera->getTagOffset() > FLAG_IN_CENTER)
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, -FOLLOW_ANGULAR);
    }
    //if the flag is to the left of the camera turn left
    else if (readCamera->getTagOffset() < -FLAG_IN_CENTER)
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, FOLLOW_ANGULAR);
    }
  return;
}
