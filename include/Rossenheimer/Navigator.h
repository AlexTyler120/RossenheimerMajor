#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>
#include "Sensors/Sensor.h"
#include "Goal/Goal.h"
#include "Goal/GoalBased.h"
#include "Goal/GoalFire.h"
#include "Goal/GoalFlood.h"
#include <vector>
#include <iostream>

class Navigator {
    public:
        Navigator();
        ~Navigator();
        void MoveToGoal(int GoalNum);
        void SetGoal(double x, double y, double pose, int goalType);  
    private:
        std::vector<std::pair< Goal *, int>> Goals;
        int GoalNum;
        GoalBased* BaseGoal;
        GoalFire* FireGoal;
        GoalFlood* FloodGoal;
};

#endif