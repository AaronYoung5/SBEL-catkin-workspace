#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

void callback(const std_msgs::Float32MultiArray::ConstPtr &array) {
  // ROS_INFO(array->data);

  float Arr[3];
  int i = 0;
  for (std::vector<float>::const_iterator it = array->data.begin();
       it != array->data.end(); ++it) {
    Arr[i] = *it;
    i++;
  }

  std::cout << "(" << Arr[0] << ", " << Arr[1] << ", " << Arr[2] << ")"
            << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_listener");
  ros::NodeHandle n;
  ros::Subscriber sub =
      n.subscribe<std_msgs::Float32MultiArray>("veh_gps", 1000, callback);

  ros::spin();
}
