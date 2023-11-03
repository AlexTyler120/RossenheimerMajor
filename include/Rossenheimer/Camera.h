#ifndef CAMERA_H
#define CAMERA_H


// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

//---Camera Implementation------------------------------------------------------
// The Camera class creates a publisher and subscriber for the camera module,
// handles image processing within the imageCallback function, and contains 
// additional getter functions to return information about detected April Tags.

class Camera
{
  public:
    //--Constructors and Destructors--
    Camera();
    Camera(ros::NodeHandle& nh_);
    ~Camera();

    //--Callback Function--
    // function subscribes to node topic and processes / outputs image data
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    //--Getter Functions--
    bool getTagDetected();    // returns true if tag is detected
    double getTagOffset();    // returns offset from tag
    int getTagID();           // returns tag ID

private:
    //--Image Subscriber--
    image_transport::Subscriber sub_;

    //--Image Publisher--
    ros::Publisher pub_;

    //--Member variables storing april tag data--
    double m_tag_offset = 0.0;      // tag offset from middle of camera
    bool m_tag_detected = false;    // true if tag is detected
    int m_tag_id;                   // id of detected tag
};


  //--Camera Constants--
  // used in the publisher and subscriber
  const std::string IMAGE_TOPIC = "/camera/image";        // use for simulation, change to "/camera/image_raw" for hardware
  const std::string PUBLISH_TOPIC = "/image_converter/output_video";


#endif // CAMERA_H