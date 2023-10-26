#ifndef GOALFIRE_H_
#define GOALFIRE_H_

#include <ros/ros.h>
#include "Goal.h"

class GoalFire : public Goal
{
    public:
        GoalFire(double px, double py, double pz, double ox, double oy, double oz, double ow, int type, int id);
        void actionTask();
        ~GoalFire();
    private:
};

#endif /* GOALFIRE_H_ */