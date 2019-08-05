#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
private:
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;

  sensor_msgs::Image &image;

public:
  ImageConverter(ros::NodeHandle &n);
  ~ImageConverter() { cv::destroyWindow(OPENCV_WINDOW); }

  sensor_msgs::Image &image() { return image; }

private:
  void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
};
