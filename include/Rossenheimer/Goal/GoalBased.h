#ifndef BASED_H_
#define BASED_H_

#include "Rossenheimer/Goal/Goal.h"

class Base : public Goal
{
    public:
        Base();
        void actionTask();
        ~Base();
    private:
};

#endif
