#ifndef BASED_H_
#define BASED_H_

#include "Goal.h"

class GoalBased : public Goal
{
    public:
        GoalBased();
        GoalBased(double coord_x, double coord_y, double orientation, int type);
        void actionTask();
        ~GoalBased();
    private:
};

#endif
