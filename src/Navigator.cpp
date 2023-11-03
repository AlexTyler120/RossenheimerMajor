// Includes relevant to msgs, publishers, and 
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

// Include necessary files
#include "../include/Rossenheimer/Navigator.h"
#include "../include/Rossenheimer/Goal/Goal.h"
#include "../include/Rossenheimer/Goal/GoalBased.h"
#include "../include/Rossenheimer/Goal/GoalFire.h"
#include "../include/Rossenheimer/Goal/GoalFlood.h"

// Include relevant standard libraries
#include <vector>

//--Constructor--
Navigator::Navigator(ros::NodeHandle& nh_): ac_("move_base", true)
{
  ROS_INFO("[CTor]: Navigator.");
  
  // instantiate the publisher for move_base
  goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
}

//--Destructor--
Navigator::~Navigator()
{
    ROS_INFO("[DTor]: Navigator.");
}


//--CheckPriorityBook()
// Determines whether the _priorityBook is empty or not.
// If empty - all incidents have been treated by TB3
bool Navigator::CheckPriorityBook()
{

  // assume empty
  bool empty = true;

  // iterate through _priorityBook
  for (int i = 0; i < BASE; i++)
  {
    if (_priorityBook[i].size() > 0)
    {
      empty = false;  // set bool false if size > 0 of any vector
    }
  }

  return empty;       // return status
}

//--Algorithm()--
// This function determines what route the TB3 should take, depending on which
// incidents are remaining in _priorityBook and what their correpsonding priorities
// and types are. From this it will resupply the TB3 with appropriate levels of 
// fire extinguisher and sandbags. This function also moves pointers to these
// incidents from the _priorityBook to _addresses. 

// Note the TB3 has a capacity of 3 items, so the only combination of routes it can 
// take are:

// 1) Prio. 0 (3 items)
// 2) Prio. 1 (2 items), Prio. 2 (1 item)
// 3) Prio. 2 (1 item), Prio. 2 (1 item), Prio. 2 (1 item)

void Navigator::Algorithm()
{
  ROS_INFO("ENTERED: ALGORITHM");

  // Array for {extinguisher, sandbags}
  int resupply[2] = {0, 0};
  
  // Checking the size of _priorityBook each time
  // This allows us to ensure the number of incidents is decreasing as they are getting
  // treated.
  ROS_INFO("_ids size: %lu", _ids.size());
  ROS_INFO("_priorityBook[0] size: %lu", _priorityBook[PRIORITY0].size());
  ROS_INFO("_priorityBook[1] size: %lu", _priorityBook[PRIORITY1].size());
  ROS_INFO("_priorityBook[2] size: %lu", _priorityBook[PRIORITY2].size());

  // CASE 1): Prio 0. incident
  if (_priorityBook[PRIORITY0].size() > 0)
  {
    // Store pointer in _addresses
    _addresses.push_back(_priorityBook[PRIORITY0].front());
    
    // Resupply 3 (extinguisher/ sandbag) based on type
    resupply[_priorityBook[PRIORITY0].front()->GetType() - 1] = 3;

    // Erase pointer from _priorityBook
    _priorityBook[PRIORITY0].erase(_priorityBook[PRIORITY0].begin(), _priorityBook[PRIORITY0].begin()+1);

  }

  // CASE 2): No Prio. 0 incidents
  else if (_priorityBook[PRIORITY1].size() > 0)
  {
    // Store Prio. 1 in _addresses
    _addresses.push_back(_priorityBook[PRIORITY1].front());
    
    // Append 2 items (extinguisher/ sandbags) to resupply array
    resupply[_priorityBook[PRIORITY1].front()->GetType() - 1] += 2;

    // Erase pointer from _priorityBook
    _priorityBook[PRIORITY1].erase(_priorityBook[PRIORITY1].begin(), _priorityBook[PRIORITY1].begin()+1);

    // With 1 spare item (of 3), check if any Prio. 2 incidents still exist
    if (_priorityBook[PRIORITY2].size() > 0)
    {
      // Store pointer in _addresses, following Prio. 1 incident
      _addresses.push_back(_priorityBook[PRIORITY2].front());

      // Append 1 item (extinguisher/ sandbags) to resupply array
      resupply[_priorityBook[PRIORITY2].front()->GetType() - 1] += 1;

      // Erase pointer from _priorityBook
      _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin(), _priorityBook[PRIORITY2].begin()+1);
    }
  }

  // Case 3): Only Priority. 2 incidents.
  else if (_priorityBook[PRIORITY2].size() > 0)
  {

    // If more than 3 Prio. 2 incidents exist:
    if (_priorityBook[PRIORITY2].size() > 3)
    {
      // Conduct same process as previous method for next 3 Prio. 2 incidents
      for (int i = 0; i < 3; i++)
      {
        _addresses.push_back(_priorityBook[PRIORITY2].front());

        // resupply accordingly.
        resupply[_priorityBook[PRIORITY2][i]->GetType() -1] += 1;
        _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin(), _priorityBook[PRIORITY2].begin()+1);

      }
    }

    // if there are less than 3 Prio. 2 incidents remaining
    else
    {
      // Append all remaining incidents to _addresses, and remove them from _priorityBook
      for (int i = 0; i < _priorityBook[PRIORITY2].size(); i++)
      {
        _addresses.push_back(_priorityBook[PRIORITY2].front());

        // resupply accordingly. 
        resupply[_priorityBook[PRIORITY2][i]->GetType() -1] += 1;
        _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin(), _priorityBook[PRIORITY2].begin()+2);
      }
    }
  }

  ROS_INFO("TB3 Resupplying %d extinguishers, %d sandbags!", resupply[0], resupply[1]);

  return;
  
}


//--GetAddress()
std::vector<Goal*> Navigator::GetAddress()
{
  return _addresses;  // return the vector of incidents on upcoming route
}

//--GetBase()--
Goal* Navigator::GetBase()
{
  return _priorityBook[BASE].front();   // return pointer to Base
}

//--SetBase()--
// Sets the position (xyz) and orientation (xyzw) of TB3 at its base,
// in the GoalBased object stored in Prio. 4 in _priorityBook.
void Navigator::SetBase()
{
  // Construct GoalBased from desired location and orientation
  Goal* base = new GoalBased(BASE_PX, BASE_PY, BASE_PZ, BASE_OX, BASE_OY, BASE_OZ, BASE_OW, TYPE_BASE, BASE_ARPIL);
  
  // Add base to Priority 3 (special case) - solely for the Base
  _priorityBook[BASE].push_back(base); 
}

//--SetGoal()--
// This function takes in april tag id, position (xyz), and orientation (xyzw)
// and appends to the relevant vectors in Navigator - to later construct the 
// incidents from
void Navigator::SetGoal(int aprilId, double x, double y, double z, double ox, double oy, double oz, double ow)
{
  _ids.push_back(aprilId);                     // append to april tag ids vector
  _position.push_back({x, y, z});               // append to position vector
  _orientation.push_back({ox, oy, oz, ow});     // append to orientation vector
}

//--PrintBook()--
// This prints the size of each priority vector
void Navigator::PrintBook()
{
  ROS_INFO("PRIO0-SIZE %lu", _priorityBook[PRIORITY0].size());    // Prio. 0
  ROS_INFO("PRIO1-SIZE %lu", _priorityBook[PRIORITY1].size());    // Prio. 1
  ROS_INFO("PRIO2-SIZE %lu", _priorityBook[PRIORITY2].size());    // Prio. 2
}

//--FindTag()--
// This function checks if a specific id (tag) already exists in the _ids vector
// It is used in Controller.FrontierDetection(), to prevent duplicates of the same
// disaster being made.

bool Navigator::FindTag(int tag)
{
  // default found status to false:
  bool found = false;

  // iterate through _ids
  for (int i = 0; i < _ids.size(); i++)
  {
    // if tag id exists
    if (_ids[i] == tag)
    {
      ROS_INFO("FOUND MATCHING ID");
      found = true;     // found = true
    }
  }

  return found;   // return found status
}

//--SortGoals()--
// This function instantiates each disaster from its relevant properties as per
// _ids, _position, and _orientation vectors. 
// It then stores a pointer to each incident in the corresponding vector from 
// _priorityBook.

void Navigator::SortGoals()
{
  // iterate through all ids foudn
  for (int i = 0; i < _ids.size(); i++)
  {

    // Construct pointer to Goal for each incident
    Goal* pGoal;

    //--Goal Data--
    // Tag ID
    int goal_id = _ids[i];              
    
    // Position x, y, z
    double point_x = _position[i][0];   
    double point_y = _position[i][1];
    double point_z = _position[i][2];

    // Orientation x, y, z, w
    double o_x = _orientation[i][0];
    double o_y = _orientation[i][1];
    double o_z = _orientation[i][2];
    double o_w = _orientation[i][3];
    

    int ans = floor(_ids[i]/PRIORITY_INDEX_SIZE);               // priority of the incident
    int temp_id = _ids[i]- (PRIORITY_INDEX_SIZE * ans);         // id of the incident
    
    // Construct Fire if id <= 50
    if (temp_id <= PRIORITY_DIVIDER)
    {
      pGoal = new GoalFire(point_x, point_y, point_z, o_x, o_y, o_z, o_w, TYPE_FIRE, temp_id);
    }

    // Construct Flood if id > 50
    else
    { 
      pGoal = new GoalFlood(point_x, point_y, point_z, o_x, o_y, o_z, o_w, TYPE_FIRE, temp_id);
    }  

    // Store pointer to incident in relevant priority vector
    _priorityBook[ans].push_back(pGoal);      
  }

  // This loop simply prints incident in each priority vector
  for (int i= 0; i < BASE; i++)
  {
    for (int j = 0; j < _priorityBook[i].size(); j++)
    {
      double x = _priorityBook[i][j]->GetPosition(0);
      double y =  _priorityBook[i][j]->GetPosition(1);
      int Type = _priorityBook[i][j]->GetPosition(7);
      ROS_INFO("Prio: %d, Elem: %d, X: %f, Y: %f, Type: %d", i, j, x, y, Type);
    }
  }

  return;
}


//--MoveToGoal()--
// This function utilises geometry_msgs to construct a target (with position and orientation)
// in conjunction with move_base and the actionlib::SimpleClientGoalState to get the TB3, to
// navigate to a desired Goal/ incident location. It takes in a pointer to an incident/ goal.

void Navigator::MoveToGoal(Goal* mvPoint)
{  

  // instantiate target of TB3
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();

  // Store position of incident in target
  goal.pose.position.x = mvPoint->GetPosition(0);
  goal.pose.position.y = mvPoint->GetPosition(1);
  goal.pose.position.z = mvPoint->GetPosition(2);

  // Store orientation at incident in target
  goal.pose.orientation.x = mvPoint->GetPosition(3);
  goal.pose.orientation.y = mvPoint->GetPosition(4);   
  goal.pose.orientation.z = mvPoint->GetPosition(5);
  goal.pose.orientation.w = mvPoint->GetPosition(6);

  // Publish the msg. 
  goal_pub.publish(goal);  

  // Wait for the action server to come up
  while (!ac_.waitForServer(ros::Duration(5.0)))
  {
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Send the goal to the action server
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose = goal;
  ac_.sendGoal(mb_goal);

  // Wait for the result
  ac_.waitForResult();

  // Check if the goal was reached
  if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    mvPoint->actionTask();
    ROS_INFO("The robot has reached the goal");
  }
  else
  {
    ROS_INFO("The robot failed to reach the goal");
  }
}