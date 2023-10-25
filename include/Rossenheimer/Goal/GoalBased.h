#ifndef BASED_H_
#define BASED_H_

#include "Goal.h"

class GoalBased : public Goal
{
    public:
        GoalBased();
        GoalBased(double coord_x, double coord_y, double orientation);
        void actionTask();
        ~GoalBased();
    private:
};

#endif