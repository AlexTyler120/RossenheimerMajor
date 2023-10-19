#include "../include/Rossenheimer/Camera.h"


Camera::Camera() {}

Camera::Camera(ros::NodeHandle& nh_)
{
  // creating a ROS subscriber for the input image
  image_transport::ImageTransport it(nh_);
  sub_ = it.subscribe(IMAGE_TOPIC, 1, &Camera::imageCallback, this);

  // creating a ROS publisher for the output image
  pub_ = nh_.advertise<sensor_msgs::Image>(PUBLISH_TOPIC, 1);

  // create opencv window
  cv::namedWindow("Detected Markers");
}

Camera::~Camera()
{
  cv::destroyWindow("Detected Markers");
}

void Camera::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Image incoming");
  cv_bridge::CvImagePtr cvPointer;

  try
  {
      cvPointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
  }

  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  std::vector<int> markerIds;
  cv::aruco::detectMarkers(cvPointer->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  cv::Mat outputImage = cvPointer->image.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  m_tag_detected = false;

// Calculate the horizontal distance from the center of the detected tag from the center of the camera image
  if (!markerIds.empty())
  {
    m_tag_detected = true;
    int markerIndex = 0; // Use the first detected marker
    cv::Moments moments = cv::moments(markerCorners[markerIndex]);
    double cx = moments.m10 / moments.m00;
    int imageWidth = cvPointer->image.cols;
    int centerX = imageWidth / 2;
    double dx = cx - centerX;
    m_tag_offset = dx;
    ROS_INFO("Horizontal distance from center: %f", dx);
  }

  ROS_INFO("Tag detected: %d", m_tag_detected);

  cv::imshow("Detected Markers", outputImage);
  cv::waitKey(1);
}

bool Camera::getTagDetected()
{
    return m_tag_detected;
}

double Camera::getTagOffset()
{
    return m_tag_offset;
}