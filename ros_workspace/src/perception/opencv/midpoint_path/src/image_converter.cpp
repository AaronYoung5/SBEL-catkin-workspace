#include "midpoint_path/image_converter.h"

ImageConverter::ImageConverter(ros::NodeHandle &n)
    : control_pub_(n.advertise<common_msgs::Control>("control", 1)) {
  // Subscrive to input video feed and publish output video feed
  image_transport::ImageTransport it(n);

  sub_ = it.subscribe("/image_transport/camera", 1,
                      &ImageConverter::imageCallback, this);
  pub_ = it.advertise("image_visualizer", 1);

  // cv::namedWindow(ORIGINAL_WINDOW);
  // cv::namedWindow(THRESH_WINDOW);
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
  // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

  // Update GUI Window
  cv::flip(cv_ptr->image, cv_ptr->image, -1);
  if (should_imshow)
    cv::imshow(ORIGINAL_WINDOW, cv_ptr->image);

  cv::Mat image_hsv;

  cv::cvtColor(cv_ptr->image, image_hsv, cv::COLOR_BGR2HSV);

  cv::Mat1b green_mask, red_mask;

  // CHRONO SETTINGS

  // Detects GREEN :: L(50, 50, 50) -> H(175, 175, 175)
  // Also Detects GREEN :: L(50, 50, 50) -> H(75, 255, 255)
  // Green has H value around 60
  cv::inRange(image_hsv, cv::Scalar(50, 50, 0), cv::Scalar(75, 255, 255),
              green_mask);

  // Detects RED :: L(0, 50, 0) -> H(2, 200, 255)
  // Red has H value around 0
  cv::inRange(image_hsv, cv::Scalar(0, 50, 0), cv::Scalar(2, 200, 255),
              red_mask);

  // REAL LIFE SETTINGS SETTINGS

  // Detects GREEN :: L(50, 50, 50) -> H(175, 175, 175)
  // Also Detects GREEN :: L(50, 50, 50) -> H(75, 255, 255)
  // Green has H value around 60
  // cv::inRange(image_hsv, cv::Scalar(70, 200, 50), cv::Scalar(90, 255, 255),
  // green_mask);
  // rgb(25, 213, 149)
  // Detects RED :: L(0, 50, 0) -> H(2, 200, 255)
  // Red has H value around 0
  // cv::inRange(image_hsv, cv::Scalar(0, 200, 50), cv::Scalar(7, 255, 255),
  // red_mask);

  cv::Mat1b mask = green_mask | red_mask;

  if (should_imshow)
    cv::imshow(THRESH_WINDOW, mask);

  cv::Mat1b green_canny_output;
  cv::Mat1b red_canny_output;

  // Find green cones
  cv::Canny(green_mask, green_canny_output, 1, 1);
  std::vector<std::vector<cv::Point>> green_contours;
  findContours(green_canny_output, green_contours, cv::RETR_TREE,
               cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> green_contours_poly(
      green_contours.size());
  std::vector<cv::Rect> green_cones;
  for (size_t i = 0; i < green_contours.size(); i++) {
    cv::approxPolyDP(green_contours[i], green_contours_poly[i], 3, true);
    cv::Rect temp = cv::boundingRect(green_contours_poly[i]);
    if (temp.area() > 500) {
      green_cones.push_back(temp);
    }
  }

  // Find red cones
  cv::Canny(red_mask, red_canny_output, 1, 1);
  std::vector<std::vector<cv::Point>> red_contours;
  findContours(red_canny_output, red_contours, cv::RETR_TREE,
               cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> red_contours_poly(red_contours.size());
  std::vector<cv::Rect> red_cones;
  for (size_t i = 0; i < red_contours.size(); i++) {
    cv::approxPolyDP(red_contours[i], red_contours_poly[i], 3, true);
    cv::Rect temp = cv::boundingRect(red_contours_poly[i]);
    if (temp.area() > 100) {
      red_cones.push_back(temp);
    }
  }

  cv::Scalar yellow = cv::Scalar(0, 255, 255);
  cv::Scalar blue = cv::Scalar(255, 0, 0);
  cv::Scalar green = cv::Scalar(0, 255, 0);

  int num = 1000;
  float n = .5;

  /**Triangle tri(cv::Point(-num, msg->height),
               cv::Point(msg->width + num, msg->height),
               cv::Point(msg->width / 2, msg->height * n));

  int thickness = 2;
  int linetype = 8;
  cv::line(cv_ptr->image, tri.x, tri.y, yellow, thickness, linetype);
  cv::line(cv_ptr->image, tri.y, tri.z, yellow, thickness, linetype);
  cv::line(cv_ptr->image, tri.z, tri.x, yellow, thickness, linetype);**/

  common_msgs::Control control;

  if (green_cones.size() == 0 && red_cones.size() == 0) {
    control.throttle = 0;
    control.steering = 0;
    control.braking = 1;
  } else if (green_cones.size() == 0) {
    control.throttle = 0.15;
    control.steering = .6;
  } else if (red_cones.size() == 0) {
    control.throttle = 0.15;
    control.steering = -.6;
  } else {
    cv::Point avgG((green_cones[0].tl() + green_cones[0].br()) / 2);
    for (size_t i = 1; i < green_cones.size(); i++) {
      cv::Point holdG((green_cones[i].tl() + green_cones[i].br()) / 2);
      if (holdG.y > avgG.y) {
        avgG = holdG;
      }
    }
    cv::Point avgR((red_cones[0].tl() + red_cones[0].br()) / 2);
    for (size_t i = 1; i < red_cones.size(); i++) {
      cv::Point holdG((red_cones[i].tl() + red_cones[i].br()) / 2);
      if (holdG.y > avgR.y) {
        avgR = holdG;
      }
    }
    cv::Point destination((avgR + avgG) / 2);
    cv::Point center(msg->width / 2, msg->height / 2);
    control.steering = (destination.x - center.x) / (float)center.x;

    cv::circle(cv_ptr->image, destination, 10, blue, 5, 8, 0);

    for (size_t i = 0; i < green_cones.size(); i++) {
      cv::rectangle(cv_ptr->image, green_cones[i].tl(), green_cones[i].br(),
                    green, 2);
    }

    for (size_t i = 0; i < red_cones.size(); i++) {
      cv::rectangle(cv_ptr->image, red_cones[i].tl(), red_cones[i].br(), green,
                    2);
    }
    control.throttle = .15;
  }

  if (should_imshow)
    cv::imshow(OBJ_WINDOW, cv_ptr->image);

  cv::waitKey(3);

  // Output modified video stream
  // pub_.publish(cv_ptr->toImageMsg());
  clamp(control);
  control_pub_.publish(control);
}

void ImageConverter::clamp(common_msgs::Control &control) {
  control.throttle =
      control.throttle > 1 ? 1 : control.throttle < 0 ? 0 : control.throttle;
  control.steering =
      control.steering > 1 ? 1 : control.steering < -1 ? -1 : control.steering;
}

float ImageConverter::areaTriangle(Triangle tri) {
  return abs((tri.x.x * (tri.y.y - tri.z.y) + tri.y.x * (tri.z.y - tri.x.y) +
              tri.z.x * (tri.x.y - tri.y.y)) /
             2.0f);
}

bool ImageConverter::isInside(cv::Point pt, Triangle tri) {
  float A0 = areaTriangle(tri);

  float A1 = areaTriangle(Triangle(pt, tri.y, tri.z));

  float A2 = areaTriangle(Triangle(pt, tri.x, tri.z));

  float A3 = areaTriangle(Triangle(pt, tri.x, tri.y));

  return (A0 == A1 + A2 + A3);
}
