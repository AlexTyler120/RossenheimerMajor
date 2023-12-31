// Includes
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// Include header files
#include "../include/Rossenheimer/TurtleBot3.h"


//--TurtleBot3 Constructor Implementation----------------------------
// Use dynamic memory to construct member objects within TurtleBot3
TurtleBot3::TurtleBot3(ros::NodeHandle& nh_)
{
    // construct Lidar, Odometer and Camera
    TurtleBotSensors[LIDAR_INDEX] = new Lidar(nh_);
    TurtleBotSensors[ODOMETER_INDEX] = new Odometer(nh_);

    _motor = new Motor(nh_);           // construct Motor
    _controller = new Controller(TurtleBotSensors[ODOMETER_INDEX], nh_); // construct Controller
    _camera = new Camera(nh_);        // construct Camera

    //to view in rviz
    // Create a Marker message to draw the path
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.05; // Width of the line
    path_marker.color.a = 1.0; // Alpha (1.0 is fully opaque)
    path_marker.color.r = 1.0; // Red
    path_marker.color.g = 0.0; // Green
    path_marker.color.b = 0.0; // Blue
    marker_pub = nh_.advertise<visualization_msgs::Marker>("path_marker", 10);

}

//--TurtleBot3 Destructor Implementation----------------------------
// Use dynamic memory to free memory created from 
TurtleBot3::~TurtleBot3()
{
    // free memory from Lidar and Odometer
    delete TurtleBotSensors[LIDAR_INDEX];
    delete TurtleBotSensors[ODOMETER_INDEX];

    delete _motor;          // free memory from Motor
    delete _controller;     // free memory from Controller
    delete _camera;         // free memory from Camera
}


void TurtleBot3::UpdateTank(int* depot_array)
{
    sandbag_payload = depot_array[0];
    spray_payload = depot_array[1];

    ROS_INFO("Resupplying TB3: %d sandbags, %d spray", sandbag_payload, spray_payload);
}

//--TurtleBot3 extinguish_world Implementation------------------------------
// Loop for ros to read data and determine appropriate manoeuver
// Call algorithm from controller of TurtleBot3
bool TurtleBot3::ExtinguishWorld()
{
    ROS_INFO("ENTERED solveMaze");

    ros::Rate loop_rate(125);   // set loop rate for ros
    while (ros::ok)             // establish loop for node to operate
    {
        // pass lidar, odometer and motor pointers into the algorithm of controller
        _controller->SaveWorld(TurtleBotSensors[LIDAR_INDEX], TurtleBotSensors[ODOMETER_INDEX], _motor, _camera);
        AddPathPoint();
        marker_pub.publish(path_marker);

        ros::spinOnce();        // spin Lidar

        loop_rate.sleep();      // sleep rate
    }
}

//--TurtleBot3 getOdom Implementation--------------------------------
double TurtleBot3::GetOdom(int req)
{
    return TurtleBotSensors[ODOMETER_INDEX]->sensorGetData(req); //Where is the turtlebot in the map
}

//--TurtleBot3 addPathPoint Implementation---------------------------
// 
void TurtleBot3::AddPathPoint()
{
    //use odom reading to generate a marker
    geometry_msgs::Point point;
    point.x = GetOdom(ODOM_X_INDEX);
    point.y = GetOdom(ODOM_Y_INDEX);
    point.z = 0.0;

    path_marker.points.push_back(point);
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
}
