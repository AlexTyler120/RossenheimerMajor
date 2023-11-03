// Header guards
#ifndef BASED_H_
#define BASED_H_

// Includes
#include <ros/ros.h>
#include "Goal.h"

//---GoalBased Implementation---------------------------------------------------------
// The GoalBased class is a child class to Goal, completing a separate actionTask
// function to other goal types. There should only be one GoalBased object for the
// duration of the program
class GoalBased : public Goal
{
    public:
        // Constructors and Destructors
        GoalBased();
        GoalBased(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        ~GoalBased();

        // Redefined virtual function from parent class
        void actionTask();
    private:      
};

#endif
