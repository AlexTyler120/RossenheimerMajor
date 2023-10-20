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

Navigator::Navigator()
{
    GoalNum = 0;
    ROS_INFO("Navigation object created");
}
Navigator::~Navigator()
{
    ROS_INFO("Navigation object destroyed");
}

void Navigator::SetGoal(double x, double y, double pose, int goalType)
{    
    if (goalType == 0)
    {
        GoalBased* BaseGoal = new GoalBased(x, y, pose);
        Goals.push_back(std::make_pair(BaseGoal, GoalNum));
    }
    else if (goalType == 1) //Fire
    {
        GoalFire* FireGoal = new GoalFire(x, y, pose);
        Goals.push_back(std::make_pair(FireGoal, GoalNum));
    }
    else if (goalType == 2) //Flood
    {
        GoalFlood* FloodGoal = new GoalFlood(x, y, pose);
        Goals.push_back(std::make_pair(FloodGoal, GoalNum));
    }
    else
    {
        ROS_INFO("Invalid goal type");
    }
    GoalNum += 1;
    
}

void Navigator::MoveToGoal(int GoalNum)
{  
  for (int i = 0; i < Goals.size(); i++)
  {
    if (Goals[i].second == GoalNum) // find specific goal
    {
      // Create a MoveBaseClient object
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);

    // Wait for the action server to come up
    while (!move_base.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Send the goal to move_base
    move_base.sendGoal(Goals[i].first->goal); //move to that goal data

    // Wait for move_base to complete the goal
    move_base.waitForResult();

    // Check if the goal was successful
    if (move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Goal reached!");
    }
    else
    {
      ROS_INFO("Failed to reach goal");
    }
    }
  }

}