#include <ros/ros.h>

#include <common_utilities/Shapes.h>
#include <common_utilities/Vector.h>

#include <common_msgs/Control.h>
#include <opencv_msgs/ConeImageMap.h>

using namespace common_utilities;

class Controller {
private:
  ros::Publisher pub_;
  ros::Subscriber sub_;

public:
  Controller(ros::NodeHandle &n);
  ~Controller() {}

private:
  void imageCallback(const opencv_msgs::ConeImageMap::ConstPtr &msg);

  void clamp(common_msgs::Control &control);
};
