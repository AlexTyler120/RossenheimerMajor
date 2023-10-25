#ifndef GOALFLOOD_H_
#define GOALFLOOD_H_

#include <ros/ros.h>
#include "Goal.h"

class GoalFlood : public Goal
{
    public:
        GoalFlood(double coord_x, double coord_y, double orientation, int type, int id);
        void actionTask();
        ~GoalFlood();
    private:
};

#endif /* GOALFLOOD_H_ */