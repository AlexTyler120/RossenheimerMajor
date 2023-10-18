#include <ros/ros.h>

#include "../include/Rossenheimer/Sensor.h"
#include "../include/Rossenheimer/Lidar.h"
#include "../include/Rossenheimer/Odometer.h"

Lidar::Lidar()
{
    std::cout << "Baited" << std::endl;
}
Lidar::Lidar(ros::NodeHandle* nh_)
{
    // sensor_name = "scan";

    laser_scan_sub_ = nh_->subscribe("scan", 10, &Lidar::sensorCallBack, this);

    escape_range_       = 70.0 * DEG2RAD;  // 90 degrees for turns
    check_forward_dist_ = 0.2; // changed from 0.7
    check_side_dist_    = 0.3; // changed from 0.6

    
}

Lidar::~Lidar()
{
    std::cout << "terminating lidar subclass" << std::endl;
}

void Lidar::sensorCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    uint16_t scan_angle[5] = {0, 90, 270, 75, 105}; //BACKLEFT and FRONTLEFT are each 15 degrees from the left position

    for (int num = 0; num < 5; num++)
    {
        if (std::isinf(msg->ranges.at(scan_angle[num])))
        {
            scan_data_[num] = msg->range_max;
        }
        else
        {
            scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }
        
        if (scan_data_[num] == 0) // when open space detects a 0 so change to 2 meter
        {
            scan_data_[num] = 2;
        }
    }
}

double Lidar::sensorGetData(int req)
{
    if (req >= 0)
    {
        return scan_data_[req];
    }
    else
    {
        return escape_range_;
    }
    
}

void Lidar::sensorSetData(double req)
{
    return;
}

