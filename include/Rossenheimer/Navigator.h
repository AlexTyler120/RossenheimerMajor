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
        void SetGoal(int april_id, double x, double y, double orientation);
        void SortGoals();
    private:
        static const int numPriorities = 3;

        std::vector<std::pair< Goal *, int>> Goals;

        std::set< std::pair<int, std::pair<std::pair<double, double>, double>>> _ids_pos;
        std::array< std::set <Goal *>, numPriorities> _priorityBook;
        /* 
            array = 
            priority 0: {}
            priority 1: {}
            priority 2: {}
        */

       // 0 - 50 : Priority 0 Fire
       // 51 - 99 : Priority 0 Flood
       // 100 - 150 : Priority 1 Fire
       // 151 - 199 : Priority 1 Flood
       // 200 - 250: Priority 2 Fire
       // 251 - 299: Priority 2 Flood

        int GoalNum;
        GoalBased* BaseGoal;
        GoalFire* FireGoal;
        GoalFlood* FloodGoal;
};

#endif