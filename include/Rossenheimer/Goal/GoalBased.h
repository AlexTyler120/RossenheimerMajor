#ifndef BASED_H_
#define BASED_H_

#include "Rossenheimer/Goal/Goal.h"

class Base : public Goal
{
    public:
        Base();
        Base(double coord_x, double coord_y, double orientation);
        void actionTask();
        ~Base();
    private:
};

#endif
