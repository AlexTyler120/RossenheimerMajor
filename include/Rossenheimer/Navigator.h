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
#include <array>
#include <iostream>
#include <cmath>

class Navigator {
    public:
        Navigator();
        ~Navigator();
        void MoveToGoal(int GoalNum);
        void SetGoal(double x, double y, double pose, int goalType);  
        void SetGoal(int april_id, double x, double y);
        void SortGoals();
    private:
        static const int numPriorities = 3;

        std::vector<std::pair< Goal *, int>> Goals;

        std::set<std::pair< int, std::pair<double, double>>> _ids_pos;
        std::array< std::set <Goal *>, numPriorities> _priorityBook;

        int GoalNum;
        GoalBased* BaseGoal;
        GoalFire* FireGoal;
        GoalFlood* FloodGoal;
};

#endif