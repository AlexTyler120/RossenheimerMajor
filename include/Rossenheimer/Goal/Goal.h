#ifndef GOAL_H_
#define GOAL_H_

#include <ros/ros.h>
#include "../Sensors/Sensor.h" 
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

class Goal
{
    public:
        Goal();
        Goal(double coord_x, double coord_y, double orientation);
        virtual void actionTask() = 0;
        ~Goal();
    protected:
        bool status;
        double Target_x;
        double Target_y;
        double Target_Orientation;
        move_base_msgs::MoveBaseGoal goal;
};

#endif /* GOAL_H_ */