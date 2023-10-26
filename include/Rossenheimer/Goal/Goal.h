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
        Goal(double coord_x, double coord_y, double pose_z, double pose_w, int type, int id);
        virtual void actionTask() = 0; //
        ~Goal();
        int GetType();
        double GetPosition(int req);
    protected:
        bool status;
        double Target_x;
        double Target_y;
        double Target_z = 0;
        double Pose_x = 0;
        double Pose_y = 0;
        double Pose_z;
        double Pose_w;

        int Target_type;
        int april_id;

};

#endif /* GOAL_H_ */