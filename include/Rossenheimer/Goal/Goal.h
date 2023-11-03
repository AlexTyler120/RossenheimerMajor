// Header Guard

#ifndef GOAL_H_
#define GOAL_H_

// Include header files
#include "../Sensors/Sensor.h" 

// Includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

// Indexes for goal types
enum
{
    TYPE_BASE = 0,
    TYPE_FIRE,
    TYPE_FLOOD
};

//---Goal Implementation---------------------------------------------------------
// The Goal class is parent to the child classes of: GoalFire, GoalFlood and
// GoalBased. The defined functions and member variables are used by the child
// classes in order to execute the desired task for the desired class type

class Goal
{
    public:
        //--Constructors and Destructors--
        Goal();
        Goal(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        virtual void actionTask() = 0; //
        ~Goal();

        //--Functionality--
        // Functions for obtaining goal type and position based on Camera and
        // Odometer readings
        int GetType();
        double GetPosition(int req);
    protected:
        // Member variables that define the goal

        // Goal status (active/inactive)
        bool status;

        // Position variables
        double Target_x;    
        double Target_y;
        double Target_z;
        double Pose_x;
        double Pose_y;
        double Pose_z;
        double Pose_w;

        // Variables used to identify goal and type
        int Target_type;
        int april_id;

};

#endif /* GOAL_H_ */
