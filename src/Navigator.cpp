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

Navigator::Navigator(ros::NodeHandle& nh_)
{
    // GoalNum = 0;
    ROS_INFO("Navigation object created");
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
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

int* Navigator::algorithm()
{
  int* resupply = new int[2];
  resupply[0] = 0;
  resupply[1] = 0;

  if (_priorityBook[PRIORITY0].size() > 0)
  {
    objectives.push_back(_priorityBook[PRIORITY0].front());
    resupply[_priorityBook[PRIORITY0].front()->GetType() - 1] = 3;
    _priorityBook[PRIORITY0].erase(_priorityBook[PRIORITY0].begin());

    ROS_INFO("Priority 0: Task Set");
  }

  else if (_priorityBook[PRIORITY1].size() > 0)
  {
     objectives.push_back(_priorityBook[PRIORITY1].front());
    resupply[_priorityBook[PRIORITY1].front()->GetType() - 1] += 2;
    _priorityBook[PRIORITY1].erase(_priorityBook[PRIORITY1].begin());

    if (_priorityBook[PRIORITY2].size() > 0)
    {
      objectives.push_back(_priorityBook[PRIORITY2].front());
      resupply[_priorityBook[PRIORITY2].front()->GetType() - 1] += 1;
      _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin());

    }
  }

  else if (_priorityBook[PRIORITY2].size() > 0)
  {
    if (_priorityBook[PRIORITY2].size() > 3)
    {
      for (int i = 0; i < 3; i++)
      {
        objectives.push_back(_priorityBook[PRIORITY2].front());
        resupply[_priorityBook[PRIORITY2][i]->GetType() -1] += 1;
        _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin());

      }
    }

    else
    {
      for (int i = 0; i < _priorityBook[PRIORITY2].size(); i++)
      {
        objectives.push_back(_priorityBook[PRIORITY2].front());
        resupply[_priorityBook[PRIORITY2][i]->GetType() -1] += 1;
        _priorityBook[PRIORITY2].erase(_priorityBook[PRIORITY2].begin());
      }
    }
  }

  return resupply;
  
}

std::vector<Goal*> Navigator::GetAddress()
{
  return objectives;

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
}

void Navigator::SortGoals()
{

  for (auto it: _ids_pos)
  {
    
    Goal* pGoal;

    int ans = floor(it.first/100);              // priority of the incident
    int temp_id = it.first - (100 * ans);       // id of the incident
    
    if (temp_id <= 50)
    {
      pGoal = new GoalFire(it.second.first.first, it.second.first.second, it.second.second, TYPE_FIRE, temp_id);
    }

    else
    {
      pGoal = new GoalFlood(it.second.first.first, it.second.first.second, it.second.second, TYPE_FLOOD, temp_id);
    }  

    _priorityBook[ans].push_back(pGoal);      
  }
}

// void Navigator::MoveToGoal(int GoalNum)
// {  
//   for (int i = 0; i < Goals.size(); i++)
//   {
//     if (Goals[i].second == GoalNum) // find specific goal
//     {
//       // Create a MoveBaseClient object
//     actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);

//     // Wait for the action server to come up
//     while (!move_base.waitForServer(ros::Duration(5.0)))
//     {
//       ROS_INFO("Waiting for the move_base action server to come up");
//     }

//     // Send the goal to move_base
//     move_base.sendGoal(Goals[i].first->goal); //move to that goal data

//     // Wait for move_base to complete the goal
//     move_base.waitForResult();

//     // Check if the goal was successful
//     if (move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//       ROS_INFO("Goal reached!");
//     }
//     else
//     {
//       ROS_INFO("Failed to reach goal");
//     }
//     }
//   }

// }

void Navigator::MoveToGoal(Goal* mvGoal)
{  
    ROS_INFO("MoveToGoal enterred.");
    // Create a goal message
    geometry_msgs::PoseStamped goal;
    ROS_INFO("MGoal made.");
    goal.header.frame_id = "map";
    ROS_INFO("map header");
    goal.header.stamp = ros::Time::now();
    ROS_INFO("stamped");
    // goal.pose.position.x = mvGoal->GetPosition()[0];
    goal.pose.position.x = -1.89343;
    ROS_INFO("pos x made " );
    // goal.pose.position.y = mvGoal->GetPosition(1);
    goal.pose.position.y = 0.107782;
    goal.pose.position.z = mvGoal->GetPosition(2);
    goal.pose.orientation.x = mvGoal->GetPosition(3);
    goal.pose.orientation.y = mvGoal->GetPosition(4);   
    goal.pose.orientation.z = mvGoal->GetPosition(5);
    // goal.pose.orientation.w = mvGoal->GetPosition(6);
    goal.pose.orientation.w = 0.309662;
    ROS_INFO("Goal instantiated");    

    // Publish the goal message
    goal_pub.publish(goal);  
    ROS_INFO("Goal publised");
}