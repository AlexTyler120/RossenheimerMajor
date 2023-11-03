// Includes
#ifndef GOALFLOOD_H_
#define GOALFLOOD_H_

// Include header files
#include <ros/ros.h>
#include "Goal.h"

//---GoalFire Implementation---------------------------------------------------------
// The GoalFlood class is a child class to Goal, completing a separate actionTask
// function to other goal types
class GoalFlood : public Goal
{
    public:
        // Constructors and Destructors
        GoalFlood(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        ~GoalFlood();

        // Redefined virtual function from parent class
        void actionTask();
        
    private:
};

#endif /* GOALFLOOD_H_ */
