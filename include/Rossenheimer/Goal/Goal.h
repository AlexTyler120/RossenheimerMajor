#ifndef GOAL_H_
#define GOAL_H_

#include "../Sensors/Sensor.h" 

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

enum
{
    TYPE_BASE = 0,
    TYPE_FIRE,
    TYPE_FLOOD
};

class Goal
{
    public:
        Goal();
        Goal(double coord_x, double coord_y, double orientation, int type, int id);
        virtual void actionTask() = 0; //
        ~Goal();
        int GetType();
        move_base_msgs::MoveBaseGoal goal;
    protected:
        bool status;
        double Target_x;
        double Target_y;
        double Target_Orientation;
        int Target_type;
        int april_id;
};

#endif /* GOAL_H_ */