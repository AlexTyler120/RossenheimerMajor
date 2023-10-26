#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
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

enum
{
    PRIORITY0 = 0,
    PRIORITY1,
    PRIORITY2
};

class Navigator {
    public:
        Navigator(ros::NodeHandle& nh_);
        ~Navigator();
        // void MoveToGoal(int GoalNum);
        void MoveToGoal(Goal* mvGoal);
        void SetGoal(int april_id, double x, double y, double z, double ox, double oy, double oz, double ow);
        void SortGoals();
        
        void SetBase(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);

        bool CheckPriorityBook();
        Goal* GetBase();
        void PrintBook();
        std::vector<Goal*> GetAddress();
        void algorithm();
        bool findTag(int tag);

    private:
        
        // Create a publisher for the goal
        ros::Publisher goal_pub;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
        
        static const int numPriorities = 3;

        // std::vector<std::pair< Goal *, int>> Goals;

        // std::vector< std::pair<int, std::pair<std::pair<double, double>, double>>> _ids_pos;
        std::vector< int > _ids;
        std::vector< std::array< double, 3 >> _position;
        std::vector < std::array< double, 4 >> _orientation;
        
        std::array< std::vector <Goal *>, numPriorities> _priorityBook;
        std::vector < Goal * > addresses;
        Goal* BaseGoal;

        // int addresses[3] = {0, 0, 0};

        /* 
            array = 
            priority 0: {closest Goal*, 2nd closest, ..., furthest} - requires 3 items to put out
            priority 1: {Fire }      requires 2 items to put out
            priority 2: {Flood, Fire, Flood, Flood }      requires 1 item
        */

       /* 
            TB3 has capacity of 3 items (fire_item, flood_item)
            1st: fix all prio0s
            2nd: fix Fire (p1) and Flood (p2)
            3rd: 1 fire, 2flood items: (put out remainder of prio2s)

       */ 

       // 0 - 50 : Priority 0 Fire
       // 51 - 99 : Priority 0 Flood

       // 100 - 150 : Priority 1 Fire
       // 151 - 199 : Priority 1 Flood

       // 200 - 250: Priority 2 Fire
       // 251 - 299: Priority 2 Flood
    
        // int GoalNum;
        // GoalFire* FireGoal;
        // GoalFlood* FloodGoal;
};

#endif