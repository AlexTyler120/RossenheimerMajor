// Includes

#include <ros/ros.h> // include ros

#include "../include/Rossenheimer/Controller.h" // Include Controller.h file


//--Constructor--
// Constructor initialises necessary variables and objects
// pertinent to its initial objective of frontier detection.
Controller::Controller(Sensor* readOdometer, ros::NodeHandle& nh_)
{
    ROS_INFO("[CTor]: Controller");
    _Navigator = new Navigator(nh_);    // instantiate navigator

    // Setting the position and orientation of Base in envrionemnt
    _Navigator->SetBase();

    // Initialise state in frontier detection - first objective
    turtlebot3_state_num = TB3_FRONTIER_DETECTION;

    // begin timer for frontier detection.
    timer = ros::Time::now().toSec();
}

//--Destructor--
// Remove navigator element
Controller::~Controller()
{
    ROS_INFO("[DTor]: Controller");
    delete _Navigator;      // delete pointer to Navigator.
}

//--FrontierDetection--
// While TB3 hasn't completed the mapping of its environment, it will execute Frontier Detection.
// In this function, it will register and store the location, and orientation of the TB3 when it
// detects each incident. It will also log each incident's priority (0,1 or 2) and type (fire / flood)
// depending on the ID assigned to its april tag
void Controller::FrontierDetection(bool tag_detected, double tag_offset, int tagID, double px, double py, double pz, double ox, double oy, double oz, double ow)
{
  
  // Log april tag, if offset is within bounds. 
  if (tag_detected && ((tag_offset < FLAG_DETECT) && (tag_offset> -FLAG_DETECT)))  // TODO: fix magic numbers
  {

    // if the tagID does not already exist:
    if (!_Navigator->FindTag(tagID))
    {
        ROS_INFO("CREATING NEW TAG");
        
        // Create an incident at the current position, orientation and store its tagID
        _Navigator->SetGoal(tagID, px, py, pz, ox, oy, oz, ow);
    }

    else
    {
      ROS_INFO("DID NOT CREATE A NEW TAG");
    }

  }
  return;
}

//--SaveWorld()--
// This function serves as a central function/ loop. It uses the enums to determine where the TB3
// should go, and what actions it should perform at each location.
void Controller::SaveWorld(Sensor* readLidar, Sensor* readOdometer, Motor* readMotor, Camera* readCamera)
{
  
  // enter switch-case
  switch(turtlebot3_state_num)
  {
    case TB3_FRONTIER_DETECTION:      // frontier detection case
      ROS_INFO("TB3_FRONTIER_DETECTION");

      // continue to execute frontier detection until environment is completely mapped. 
      if (ros::Time::now().toSec() - timer < EXPLORATION_TIME)
      {
          bool tagDetect = readCamera->getTagDetected();    // is april tag detected?
          bool tagOffset = readCamera->getTagOffset();      // is april tag close enough?
          int tagID = readCamera->getTagID();               // ID (int) of april tag
          
          // x, y, z: position of TB3 once tag identified
          double px = readOdometer->sensorGetData(2);       
          double py = readOdometer->sensorGetData(3);       
          double pz = readOdometer->sensorGetData(4);

          // x, y, z, w: orientation of TB3 once tag identified
          double ox = readOdometer->sensorGetData(5);
          double oy = readOdometer->sensorGetData(6);
          double oz = readOdometer->sensorGetData(7);
          double ow = readOdometer->sensorGetData(8);

          // pass variables into FrontierDetection function - logging the incidents during exploration
          FrontierDetection(tagDetect, tagOffset, tagID, px, py, pz, ox, oy, oz, ow);
      }
      
      // once exploration is complete, and environment is mapped
      else
      {

        _Navigator->SortGoals();              // sort all incidents based on priority and location
        turtlebot3_state_num = TB3_MOVE_BASE; // move to base to begin fire and flood solving.
      }

      break;                        // exit case



     
    case TB3_MOVE_BASE:             // case to move to base
      
      ROS_INFO("TB3_MOVE_BASE.");
      
      // navigate to stored base location
      while(!_Navigator->MoveToGoal(_Navigator->GetBase()))
      {
        _Navigator->MoveToGoal(_Navigator->GetBase());
      };      
      
      // determine needed supplies and corresponding route for TB3
      // as per priority book and remaining incidents
      _Navigator->Algorithm();    
      
      // once at goal, TB3 resupplied, route planned:
      // enter move to goal case
      turtlebot3_state_num = TB3_MOVE_GOAL;
      
      break;    //exit case




    case TB3_MOVE_GOAL:
      ROS_INFO("TB3_MOVE_GOAL");
     
      // iterate through all incidents in the planned route
      for (auto it: _Navigator->GetAddress())
      {
        // waiting until TB3 has moved to each goal in addresses
        while (!_Navigator->MoveToGoal(it))
        {
          _Navigator->MoveToGoal(it);     // move to each incident
        }
      }

      // now that incidents have been treated
      // reset the address book for the route

      _Navigator->GetAddress().clear();
      
      // if no more incidents in the world
      if (_Navigator->CheckPriorityBook())
      {
        turtlebot3_state_num = TB3_END;   // enter case to terminate program
      }
      
      // if more incidents exist:
      else
      {
        turtlebot3_state_num = TB3_MOVE_BASE;   // return to base and repeat.
      }

      break;    // exit case




    case TB3_END:
      ROS_INFO("WORLD SAVED");

      // navigate to the Base Address.
      while (!_Navigator->MoveToGoal(_Navigator->GetBase()))
      {
        _Navigator->MoveToGoal(_Navigator->GetBase());  
      }
      
      // check size of priority book (data structure of all incidents)
      // size of each priority array (0, 1, 2) should be 0.
      _Navigator->PrintBook();

      // terminate program
      ros::shutdown();
      
      break;   // exit case
  }
}


void Controller::CenterTag(Camera* readCamera, Motor* readMotor, Sensor* readLidar)
{
    //if the flag is in the centre of the camera approach
    if ((readCamera->getTagOffset() < FLAG_IN_CENTER) && (readCamera->getTagOffset() > -FLAG_IN_CENTER))
    {
      readMotor->updateCommandVelocity(OVERALL_LIMIT, STATIONARY);
      if ((readCamera->getTagOffset() < FLAG_DETECT) && (readCamera->getTagOffset() > -FLAG_DETECT))
      {
        ROS_INFO("Tag in center");
        double tag_dist = readLidar->sensorGetData(CENTER);
        if (tag_dist < TAG_LIMIT)
        {
          readMotor->updateCommandVelocity(STATIONARY, STATIONARY);
          ROS_INFO("Stopped with tag distance: %f", tag_dist);
        }
        else if (readCamera->getTagOffset() > FLAG_DETECT)
        {
          readMotor->updateCommandVelocity(OVERALL_LIMIT, -FOLLOW_ANGULAR);
        }
        //if the flag is to the right of the camera turn right
        else if (readCamera->getTagOffset() < -FLAG_DETECT)
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
