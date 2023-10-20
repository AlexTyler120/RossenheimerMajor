//Navigation

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

#include "../include/Rossenheimer/Navigation.h"
#include "../include/Rossenheimer/Goal/Goal.h"
#include "../include/Rossenheimer/Goal/GoalBased.h"
#include "../include/Rossenheimer/Goal/GoalFire.h"
#include "../include/Rossenheimer/Goal/GoalFlood.h"

Navigation::Navigation()
{
    ROS_INFO("Navigation object created");
}
Navigation::~Navigation()
{
    ROS_INFO("Navigation object destroyed");
}

void Navigation::SetGoal(double x, double y, double pose, int goalType)
{    
    if (goalType == 0)
    {
        Base BaseGoal(x, y, pose); //only have one base
    }
    else if (goalType == 1) //Fire
    {
        GoalFire* FireGoal = new GoalFire(x, y, pose);
    }
    else if (goalType == 2) //Flood
    {
        GoalFlood* FloodGoal = new GoalFlood(x, y, pose);
    }
    else
    {
        ROS_INFO("Invalid goal type");
    }
    
}

void Navigation::MoveToGoal(move_base_msgs::MoveBaseGoal goal)
{
  // Create a MoveBaseClient object
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);

  // Wait for the action server to come up
  while (!move_base.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Send the goal to move_base
  move_base.sendGoal(goal);

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