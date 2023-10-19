#ifndef CAMERA_H
#define CAMERA_H

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

class Camera
{
  public:
    Camera();
    Camera(ros::NodeHandle& nh_);
    ~Camera();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    bool getTagDetected();
    double getTagOffset();

private:
    image_transport::Subscriber sub_;

    ros::Publisher pub_;

    double m_tag_offset = 0.0;
    bool m_tag_detected = false;
};

  // const std::string IMAGE_TOPIC = "/camera/image_raw";    // use for hardware
  const std::string IMAGE_TOPIC = "/camera/image";        // use for simulation
  const std::string PUBLISH_TOPIC = "/image_converter/output_video";


#endif // CAMERA_H