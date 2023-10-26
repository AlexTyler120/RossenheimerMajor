//Navigation

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

#include "../include/Rossenheimer/Navigator.h"
#include "../include/Rossenheimer/Goal/Goal.h"
#include "../include/Rossenheimer/Goal/GoalBased.h"
#include "../include/Rossenheimer/Goal/GoalFire.h"
#include "../include/Rossenheimer/Goal/GoalFlood.h"

#include <vector>

Navigator::Navigator(ros::NodeHandle& nh_): ac_("move_base", true)
{
    // GoalNum = 0;
    ROS_INFO("Navigation object created");
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    // goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
}
Navigator::~Navigator()
{
    ROS_INFO("Navigation object destroyed");
}

// void Navigator::SetGoal(double x, double y, double pose, int goalType)
// {    //what is the flag
//     if (goalType == 0)
//     {
//         BaseGoal = new GoalBased(x, y, pose);
//         Goals.push_back(std::make_pair(BaseGoal, GoalNum));
//     }
//     else if (goalType == 1) //Fire
//     {
//         FireGoal = new GoalFire(x, y, pose);
//         Goals.push_back(std::make_pair(FireGoal, GoalNum));
//     }
//     else if (goalType == 2) //Flood
//     {
//         FloodGoal = new GoalFlood(x, y, pose);
//         Goals.push_back(std::make_pair(FloodGoal, GoalNum));
//     }
//     else
//     {
//         ROS_INFO("Invalid goal type");
//     }
//     GoalNum += 1;
    
// }

void Navigator::algorithm()
{
  ROS_INFO("ENTERED: ALGORITHM");
  int resupply[2] = {0, 0};
  
  ROS_INFO("_ids_pos size: %lu", _ids_pos.size());
  ROS_INFO("_priorityBook[0] size: %lu", _priorityBook[PRIORITY0].size());
  ROS_INFO("_priorityBook[1] size: %lu", _priorityBook[PRIORITY1].size());
  ROS_INFO("_priorityBook[2] size: %lu", _priorityBook[PRIORITY2].size());


  if (_priorityBook[PRIORITY0].size() > 0)
  {
    
    addresses.push_back(_priorityBook[PRIORITY0].front());
    ROS_INFO("ALGO: PUSHEDBACK PRIO-0");
    // resupply[_priorityBook[PRIORITY0].front()->GetType() - 1] = 3;
    // ROS_INFO("ALGO: ENTERED TO PRIO_BOOK");'
    ROS_INFO("Goal Type: %d", _priorityBook[PRIORITY0].front()->GetType());

    _priorityBook[PRIORITY0].erase(_priorityBook[PRIORITY0].begin());
    ROS_INFO("ALGO: ERASED PRIO-0");

    // ROS_INFO("Priority 0: Task Set");
  }

  else if (_priorityBook[PRIORITY1].size() > 0)
  {
    addresses.push_back(_priorityBook[PRIORITY1].front());
    ROS_INFO("ALGO: PUSHEDBACK PRIO-1");
    
    // resupply[_priorityBook[PRIORITY1].front()->GetType() - 1] += 2;
    // ROS_INFO("ALGO: ENTERED TO PRIO_BOOK");
    _priorityBook[PRIORITY1].erase(_priorityBook[PRIORITY1].begin());
    ROS_INFO("ALGO: ERASED PRIO-1");


    if (_priorityBook[PRIORITY2].size() > 0)
    {
      addresses.push_back(_priorityBook[PRIORITY2].front());
      // resupply[_priorityBook[PRIORITY2].front()->GetType() - 1] += 1;
      _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin());

    }
  }

  else if (_priorityBook[PRIORITY2].size() > 0)
  {
    if (_priorityBook[PRIORITY2].size() > 3)
    {
      for (int i = 0; i < 3; i++)
      {
        addresses.push_back(_priorityBook[PRIORITY2].front());
        // resupply[_priorityBook[PRIORITY2][i]->GetType() -1] += 1;
        _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin());

      }
    }

    else
    {
      for (int i = 0; i < _priorityBook[PRIORITY2].size(); i++)
      {
        addresses.push_back(_priorityBook[PRIORITY2].front());
        // resupply[_priorityBook[PRIORITY2][i]->GetType() -1] += 1;
        _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin());
      }
    }
  }

  return;
  
}

std::vector<Goal*> Navigator::GetAddress()
{
  return addresses;

}
Goal* Navigator::GetBase()
{
  return BaseGoal;
}

void Navigator::SetBase(double x, double y, double pose_z, double pose_w, int type, int id)
{
  ROS_INFO("FUKCING UNUONFKC NOIN .");
  BaseGoal = new GoalBased(x, y, pose_z, pose_w, TYPE_BASE, 0);  
}

void Navigator::SetGoal(int april_id, double x, double y, double orientation)
{
  _ids_pos.push_back(std::make_pair(april_id, std::make_pair(std::make_pair(x, y), orientation)));
  ROS_INFO("Size of _ids_pos: %lu", _ids_pos.size());
}

void Navigator::PrintBook()
{
  // for (auto it: _priorityBook)
  // {
  //   ROS_INFO("")
  // }
}

bool Navigator::findTag(int tag)
{
  bool found = false;
  for (auto it : _ids_pos)
  {
    if (it.first == tag)
    {
      found = true;
    }
  }

  return found;
}
void Navigator::SortGoals()
{
  ROS_INFO("ENTERED: SortGoals");
  for (auto it: _ids_pos)
  {
    Goal* pGoal;

    int ans = floor(it.first/100);              // priority of the incident
    int temp_id = it.first - (100 * ans);       // id of the incident
    
    if (temp_id <= 50)
    {
      ROS_INFO("FIRE: PRIO: %d, ID: %d, ORIENTATION: %f, TYPE: %d", ans, temp_id, it.second.second, TYPE_FIRE);
      pGoal = new GoalFire(it.second.first.first, it.second.first.second, it.second.second, TYPE_FIRE, temp_id);
    }

    else
    {
      ROS_INFO("FLOOD: PRIO: %d, ID: %d, ORIENTATION: %f, TYPE: %d", ans, temp_id, it.second.second, TYPE_FLOOD);
      pGoal = new GoalFlood(it.second.first.first, it.second.first.second, it.second.second, TYPE_FLOOD, temp_id);
    }  

    _priorityBook[ans].push_back(pGoal);      
  }
}


void Navigator::MoveToGoal(Goal* mvGoal)
{  
    // ROS_INFO("MoveToGoal enterred.");
    // Create a goal message
    geometry_msgs::PoseStamped goal;
    // ROS_INFO("MGoal made.");
    goal.header.frame_id = "map";
    // ROS_INFO("map header");
    goal.header.stamp = ros::Time::now();
    // ROS_INFO("stamped");
    // goal.pose.position.x = mvGoal->GetPosition()[0];
    goal.pose.position.x = mvGoal->GetPosition(0);
    // ROS_INFO("pos x made " );
    // goal.pose.position.y = mvGoal->GetPosition(1);
    goal.pose.position.y = mvGoal->GetPosition(1);
    goal.pose.position.z = mvGoal->GetPosition(2);
    goal.pose.orientation.x = mvGoal->GetPosition(3);
    goal.pose.orientation.y = mvGoal->GetPosition(4);   
    goal.pose.orientation.z = mvGoal->GetPosition(5);
    goal.pose.orientation.w = mvGoal->GetPosition(6);
    // goal.pose.orientation.w = 0.309662;
    // ROS_INFO("Goal instantiated");    

    // Publish the goal message
    goal_pub.publish(goal);  
    // ROS_INFO("Goal publised");

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
        ROS_INFO("The robot has reached the goal");
    }
    else
    {
        ROS_INFO("The robot failed to reach the goal");
    }
}