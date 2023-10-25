#ifndef GOALFIRE_H_
#define GOALFIRE_H_

#include <ros/ros.h>
#include "Goal.h"

class GoalFire : public Goal
{
    public:
        GoalFire(double coord_x, double coord_y, double orientation, int type, int id);
        void actionTask();
        ~GoalFire();
    private:
};

#endif /* GOALFIRE_H_ */