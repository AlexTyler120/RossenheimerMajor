#ifndef BASED_H_
#define BASED_H_

#include <ros/ros.h>
#include "Goal.h"

class GoalBased : public Goal
{
    public:
        GoalBased();
        GoalBased(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        void actionTask();
        ~GoalBased();
    private:
        
};

#endif
