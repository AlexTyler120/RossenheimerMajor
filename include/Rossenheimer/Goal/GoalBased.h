#ifndef BASED_H_
#define BASED_H_

#include <ros/ros.h>
#include "Goal.h"

class GoalBased : public Goal
{
    public:
        GoalBased();
        GoalBased(double coord_x, double coord_y, double pose_z, double pose_w, int type, int id);
        void actionTask();
        ~GoalBased();
    private:
        
};

#endif
