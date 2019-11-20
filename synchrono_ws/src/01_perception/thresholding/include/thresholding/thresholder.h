#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include "perception_msgs/ConeImageMap.h"

static const std::string ORIGINAL_WINDOW = "Original Image";
static const std::string THRESH_WINDOW = "Thresholded Image";
static const std::string CANNY_WINDOW = "Canny Image";
static const std::string OBJ_WINDOW = "Objects Detected Image";

static const std::string FILE_NAME = "/home/aaron/output2.avi";

class Thresholder {
private:
  image_transport::Subscriber sub_;

  ros::Publisher pub_;
  image_transport::Publisher cone_pub_;

  cv::VideoWriter video_;

  std::string image_topic_;

  bool image_display_, image_simulated_;

public:
  Thresholder(ros::NodeHandle &n);
  ~Thresholder() {
    cv::destroyAllWindows();
    video_.release();
  }

private:
  void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

  std::vector<cv::Rect> Threshold(cv_bridge::CvImagePtr &cv_ptr,
                                  cv::Scalar lower_hsv, cv::Scalar upper_hsv,
                                  bool invert);

  void drawDetectedCones(cv_bridge::CvImagePtr &cv_ptr,
                         std::vector<cv::Rect> cones, cv::Scalar color);

  std::vector<perception_msgs::Cone>
  CreateConeMsgArray(std::vector<cv::Rect> &rects, uint8_t color);

  cv::Scalar ScalarHSV2BGR(cv::Scalar hsv_scalar);
};
