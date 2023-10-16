#include "../include/Rossenheimer/Camera.h"


Camera::Camera() {}

Camera::Camera(ros::NodeHandle& nh_)
{
  // creating a ROS subscriber for the input image
  image_transport::ImageTransport it(nh_);
  sub_ = it.subscribe(IMAGE_TOPIC, 1, &Camera::imageCallback, this);

  // creating a ROS publisher for the output image
  pub = nh_.advertise<sensor_msgs::Image>(PUBLISH_TOPIC, 1);
  green_pixels_detected_pub_ = nh_.advertise<std_msgs::Bool>("green_pixels_detected", 1);
  num_green_pixels_pub_ = nh_.advertise<std_msgs::Int32>("num_green_pixels", 1);
  centroid_offset_pub_ = nh_.advertise<std_msgs::Int32>("centroid_offset", 1);

  // create opencv window
  cv::namedWindow(OPENCV_WINDOW);
}

Camera::~Camera()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void Camera::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // convert image from BGR to HSV
  cv::Mat hsv_image;
  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  // setting HSV color range for bright green pixels
  cv::Scalar lower_green(40, 50, 50);
  cv::Scalar upper_green(80, 255, 255);

  // thresholding image to only get bright green pixels
  cv::Mat green_mask;
  cv::inRange(hsv_image, lower_green, upper_green, green_mask);

  // find centroid of countoured region
  cv::Moments moments = cv::moments(green_mask, true);
  cv::Point centroid;
  if (moments.m00 > 0) 
  {
    centroid.x = moments.m10 / moments.m00;
    centroid.y = moments.m01 / moments.m00;
  }
  else 
  {
    centroid.x = 0;
    centroid.y = 0;
  }

  // find center of camera image 
  int image_center_x = green_mask.cols / 2;

  // calculate offset of bounded box from center of image
  int centroid_offset_x = centroid.x - image_center_x;

  m_centroid_offset = centroid_offset_x;
  std_msgs::Int32 offset_msg;
  offset_msg.data = centroid_offset_x;
  centroid_offset_pub_.publish(offset_msg);

  // finding counters of region
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // draw contours on camera image for visulation
  cv::Mat green_image = cv_ptr->image.clone();
  int total_green_area = 0;
  for (size_t i = 0; i < contours.size(); i++) 
  {
    cv::drawContours(green_image, contours, i, cv::Scalar(0, 255, 0), 2);
    total_green_area += cv::contourArea(contours[i]);
  }

  // checking if a threshold number of green pixels were detected
  int image_area = green_mask.rows * green_mask.cols;
  if (total_green_area >= image_area / 1000) {
    ROS_INFO("Green pixels detected!");
    m_green_pixels_detected = true;
    std_msgs::Bool flag;
    flag.data = true;
    green_pixels_detected_pub_.publish(flag);
  }
  else {
    ROS_INFO("No green pixels detected.");
    m_green_pixels_detected = false;
    std_msgs::Bool flag;
    flag.data = false;
    green_pixels_detected_pub_.publish(flag);
  }
  
  // publishing number of green pixels detected
  m_num_green_pixels = total_green_area;
  std_msgs::Int32 num_green_pixels;
  num_green_pixels.data = total_green_area;
  num_green_pixels_pub_.publish(num_green_pixels);

  // publishing processed image
  pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", green_image).toImageMsg());

  // update opencv window
  cv::imshow(OPENCV_WINDOW, green_image);
  cv::waitKey(3);
}

// getter implementations
bool Camera::getGreenPixelsDetected()
{
  return m_green_pixels_detected;
}

int Camera::getNumGreenPixels()
{
  return m_num_green_pixels;
}

int Camera::getCentroidOffset()
{
  return m_centroid_offset;
}