// Include corresponding header file
#include "../include/Rossenheimer/TurtleBot3.h"

// Include packages
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

//--TurtleBot3 Constructor Implementation----------------------------
// Use dynamic memory to construct member objects within TurtleBot3
TurtleBot3::TurtleBot3(ros::NodeHandle& nh_)
{
    // construct Lidar, Odometer and Camera
    TurtleBotSensors[LIDAR_INDEX] = new Lidar(nh_);
    TurtleBotSensors[ODOMETER_INDEX] = new Odometer(nh_);
    // TurtleBotSensors[ODOMETER_INDEX] = new Odometer(&nh_);
    // OdometerTest = new Odometer(nh_);

    _motor = new Motor(nh_);           // construct Motor
    // _controller = new Controller(TurtleBotSensors[ODOMETER_INDEX], &nh_);     // construct Controller
    _controller = new Controller(TurtleBotSensors[ODOMETER_INDEX], nh_);
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


void TurtleBot3::updateTank(int* depot_array)
{
    sandbag_payload = depot_array[0];
    spray_payload = depot_array[1];

    ROS_INFO("Resupplying TB3: %d sandbags, %d spray", sandbag_payload, spray_payload);
}

//--TurtleBot3 solveMaze Implementation------------------------------
// Loop for ros to read data and determine appropriate manoeuvre
// Call algorithm from controller of TurtleBot3
bool TurtleBot3::solveMaze()
{
    ROS_INFO("ENTERED solveMaze");
    // _controller->frontierDetection(_camera,TurtleBotSensors[LIDAR_INDEX], TurtleBotSensors[ODOMETER_INDEX]);

    ros::Rate loop_rate(125);   // set loop rate for ros
    while (ros::ok)             // establish loop for node to operate
    {
        // pass lidar, odometer and motor pointers into the algorithm of controller
        // _controller->MazeSolver(TurtleBotSensors[0], TurtleBotSensors[1], _motor, _camera);
        // _controller->SaveWorld(TurtleBotSensors[LIDAR_INDEX],TurtleBotSensors[ODOMETER_INDEX], _motor, _camera);
        _controller->SaveWorld(TurtleBotSensors[LIDAR_INDEX], TurtleBotSensors[ODOMETER_INDEX], _motor, _camera);
        // updateTank(_controller->getDepots());
        ROS_INFO("TURLEBOT SENSORS %f", TurtleBotSensors[ODOMETER_INDEX]->sensorGetData(1));
        addPathPoint();
        marker_pub.publish(path_marker);

        ros::spinOnce();        // spin Lidar

        loop_rate.sleep();      // sleep rate
    }

}

//--TurtleBot3 getOdom Implementation--------------------------------
double TurtleBot3::getOdom(int req)
{
    return TurtleBotSensors[ODOMETER_INDEX]->sensorGetData(req);
}

//--TurtleBot3 addPathPoint Implementation---------------------------
// 
void TurtleBot3::addPathPoint()
{
  geometry_msgs::Point point;
  point.x = getOdom(ODOM_X_INDEX);
  point.y = getOdom(ODOM_Y_INDEX);
  point.z = 0.0;

  path_marker.points.push_back(point);
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = ros::Time::now();
}
