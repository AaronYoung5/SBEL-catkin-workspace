#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include "common_msgs/Control.h"

static const std::string ORIGINAL_WINDOW = "Original Image";
static const std::string THRESH_WINDOW = "Thresholded Image";
static const std::string OBJ_WINDOW = "Objects Detected Image";

typedef cv::Point3_<cv::Point> Triangle;

class ImageConverter {
private:
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;

  ros::Publisher control_pub_;

  bool should_imshow = true;

  // sensor_msgs::Image &image;

public:
  ImageConverter(ros::NodeHandle &n);
  ~ImageConverter() {
    cv::destroyWindow(ORIGINAL_WINDOW);
    cv::destroyWindow(THRESH_WINDOW);
    cv::destroyWindow(OBJ_WINDOW);
  }

  // sensor_msgs::Image &image() { return image; }

private:
  void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

  float areaTriangle(Triangle tri);
  
  void clamp(common_msgs::Control &control);

  bool isInside(cv::Point pt, Triangle point);
};
