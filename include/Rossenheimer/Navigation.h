#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <ros/ros.h>
#include "Sensor.h"

class Navigation {
    public:
        Navigation();
        ~Navigation();
        void MoveToGoal(int goal);
        
    private:
        void SetGoal(Sensor* readOdometer);
        double Target_x;
        double Target_y;
        double Target_Orientation;
};

#endif