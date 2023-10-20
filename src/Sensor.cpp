// Includes
#include <ros/ros.h>    // include ros

// Include header files
#include "../include/Rossenheimer/Sensors/Sensor.h"

//--Sensor Constructor--
Sensor::Sensor()
{
    std::cout << "[CTor]: Sensor" << std::endl;
}

//--Sensor Destructor--
Sensor::~Sensor()
{
    std::cout << "[DTor]: Sensor" << std::endl;
}

//--Sensor sensorCallBack--
// Virtual function for subscribers of Sensors
void Sensor::sensorCallBack()
{

}

//--Sensor sensorSetData--
// Virtual function for setting parameters of Sensors
void Sensor::sensorSetData(double req)
{

}