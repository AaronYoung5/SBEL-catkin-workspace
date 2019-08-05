#include "cone_detection/image_converter.h"

ImageConverter(ros::NodeHandle &n) {
  // Subscrive to input video feed and publish output video feed
  image_transport::ImageTransport it(n);

  sub_ = it.subscribe("/image_transport/camera", 1,
                      &ImageConverter::imageCallback, this);
  pub_ = it.advertise("image_visualizer", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

  // Update GUI Window
  cv::flip(cv_ptr->image, cv_ptr->image, 0);
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}
