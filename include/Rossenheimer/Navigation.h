#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>
#include "Sensor.h"
#include "Goal/Goal.h"

class Navigation {
    public:
        Navigation();
        ~Navigation();
        void MoveToGoal(move_base_msgs::MoveBaseGoal goal);
        void SetGoal(double x, double y, double pose, int goalType);  
    private:
};

#endif