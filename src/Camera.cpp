// Include header files
#include "../include/Rossenheimer/Camera.h"

//--Camera Constructor--
Camera::Camera() 
{
  ROS_INFO("[CTor]: Camera.");
}

//--Camera Nodehandle Constructor--
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

//--Camera Destructor--
Camera::~Camera()
{
  cv::destroyWindow("Detected Markers");
}

//--Image Callback Function--
// Uses OpenCV to identify April Tags in the camera image, and then stores
// the tag ID and horizontal offset from the center of the camera image.
void Camera::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // creating an OpenCV pointer to the image message
  cv_bridge::CvImagePtr cvPointer;

  try
  {
    // copy the ROS image message to 'msg' 
    cvPointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    // error checking
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // defining the type of April Tag to detect (36h11) and its parameters
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

  // creating vectors the store the detected marker corners and any rejected tags
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  
  // vector to store all the detected marker IDs
  std::vector<int> markerIds;

  // detect the April Tag and store relevant information in the vectors
  cv::aruco::detectMarkers(cvPointer->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  // highlighting the detected tag and displaying information about it on the camera image
  cv::Mat outputImage = cvPointer->image.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  // resetting member variable
  m_tag_detected = false;

  //--Processing when a tag is detected--
  if (!markerIds.empty())
  {
    // tag is detected when markerIds is not empty
    m_tag_detected = true;

    // store the latest detected tag ID in the member variable m_tag_id
    int tagID = markerIds[markerIds.size()-1];
    m_tag_id = tagID;

    //--Calculate the horizontal distance from the center of the detected tag from the center of the camera image-- 
    // image must first be greyscaled for the cv::moments function to work
    cv::Mat grayImage;  

    // greyscaling the image
    if (cvPointer->image.channels() == 3)
    {
      cv::cvtColor(cvPointer->image, grayImage, cv::COLOR_BGR2GRAY);    // convert to greyscale if image is RGB
    }
    else if (cvPointer->image.channels() == 4)
    {
      cv::cvtColor(cvPointer->image, grayImage, cv::COLOR_BGRA2GRAY);   // convert to greyscale if image is RGBA
    }
    else
    {
      grayImage = cvPointer->image; // if image is already greyscaled
    }

    //--Finding the centre of the current tag--
    // creating a moment object to store the moments of the greyscaled April Tag
    cv::Moments moments = cv::moments(grayImage, true);

    // calculating the x-coordinate of the centre of the tag and center of the camera image
    double cx = moments.m10 / moments.m00;
    int imageWidth = cvPointer->image.cols;
    int centerX = imageWidth / 2;

    // finding the horizontal distance between the centre of the tag and the centre of the camera image
    double dx = cx - centerX;

    // storing the horizontal distance in the member variable m_tag_offset
    m_tag_offset = dx;
  }

  // displaying the camera image with the detected tag highlighted in a window
  cv::imshow("Detected Markers", outputImage);
  cv::waitKey(1);
}

//---Getter Functions---------------------------------------------------------------------------------------------------

// getTagDetected returns true if a tag is detected
bool Camera::getTagDetected()
{
    return m_tag_detected;
}

// getTagOffset returns the horizontal distance from the center of the detected tag from the center of the camera image
double Camera::getTagOffset()
{
    return m_tag_offset;
}

// getTagID returns the ID of the detected tag
int Camera::getTagID()
{
  return m_tag_id;
}