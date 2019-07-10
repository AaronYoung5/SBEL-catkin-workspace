// ROS include
#include "ros/ros.h"

// Package includes
#include "pid_control/PID.h"

class PIDControl {
private:
  // Private Variables

  // PID Speed Controller
  PID speed_controller_;
  // PID Steering Controller
  PID steering_controller_;

  // Controls publisher
  ros::Publisher controls_pub_;
  // Point vehicle is following publisher for debugging purposes
  ros::Publisher point_following_pub_;

  // Cone map subscriber
  // ros::Subscriber cone_sub_;
  // Vehicle state subscriber
  ros::Subscriber state_sub_;

  // Path service server
  ros::ServiceServer path_srv_;

  // Cone service client
  ros::ServiceClient cone_client_;

  // ConeMap message
  common_msgs::ConeMap cones_;
  // Control message
  common_msgs::Control control_;

  // Controls values
  float throttle_, braking_, steering_;

  // Path vector
  std::vector<Vector3D<>> path_;

public:
  // Public Methods
  PIDControl(NodeHandle n);

private:
  // Private Methods

  // Initialization of class
  void initialize();

  // Publish controls message
  void publishControls();

  // Clamping function for the control values
  void clampControls();

  // Publish point following message
  void pointFollowingPublish(Vector3D<> pos);

  // Cone map subscriber callback
  void coneCallback(const common_msgs::ConeMap::ConstPtr &msg);
  // Vehicle state subscriber callback
  void stateCallback(const common_msgs::VehState::ConstPtr &msg);

  // Path service callback
  bool pathCallback(common_srvs::Path::Request &req,
                    common_srvs::Path::Response &res);

  // Computes the minimum distance from a point to a line segment
  float minimumDistance(Vector3D<> pos, Vector3D<> begin, Vector3D<> end,
                        bool publish);
};
