// Header guards
#ifndef GOALFIRE_H_
#define GOALFIRE_H_

// Includes
#include <ros/ros.h>
#include "Goal.h"

//---GoalFire Implementation---------------------------------------------------------
// The GoalFire class is a child class to Goal, completing a separate actionTask
// function to other goal types
class GoalFire : public Goal
{
    public:
        // Constructors and Destructors
        GoalFire(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        ~GoalFire();

        // Redefined virtual function from parent class
        void actionTask();
        
    private:
};

#endif /* GOALFIRE_H_ */
