#ifndef CAMERA_H
#define CAMERA_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

class Camera
{
  public:
    Camera();
    Camera(ros::NodeHandle& nh_);
    ~Camera();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    bool getGreenPixelsDetected();
    int getNumGreenPixels();
    int getCentroidOffset();

private:
    image_transport::Subscriber sub_;

    ros::Publisher pub;
    ros::Publisher green_pixels_detected_pub_;
    ros::Publisher num_green_pixels_pub_;
    ros::Publisher centroid_offset_pub_;

    bool m_green_pixels_detected;
    int m_num_green_pixels, m_centroid_offset;
};

  const std::string IMAGE_TOPIC = "/camera/image_raw";
  const std::string PUBLISH_TOPIC = "/image_converter/output_video";
  const std::string OPENCV_WINDOW = "Image window";


#endif // CAMERA_H