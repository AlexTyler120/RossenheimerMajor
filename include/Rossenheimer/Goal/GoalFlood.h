#ifndef GOALFLOOD_H_
#define GOALFLOOD_H_

#include <ros/ros.h>
#include "Goal.h"

class GoalFlood : public Goal
{
    public:
        GoalFlood(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        void actionTask();
        ~GoalFlood();
    private:
};

#endif /* GOALFLOOD_H_ */