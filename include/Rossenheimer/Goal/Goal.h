#ifndef GOAL_H_
#define GOAL_H_

#include <ros/ros.h>
#include "../Sensors/Sensor.h" 

class Goal
{
    public:
        Goal();
        Goal(double coord_x, double coord_y, double orientation);
        virtual void actionTask() = 0;
        ~Goal();
    protected:
        bool status;
        double Target_x;
        double Target_y;
        double Target_Orientation;
};

#endif /* GOAL_H_ */